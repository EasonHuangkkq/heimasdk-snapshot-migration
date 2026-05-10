#include "rmd_can_sdk/rmd_ethercat_backend.h"

#include "rmd_can_sdk/rmd_ethercat_bindings.h"
#include "rmd_can_sdk/rmd_ethercat_mt_device.h"
#include "rmd_can_sdk/rmd_ethercat_snapshot.h"
#include "rmd_can_sdk/rmd_safety.h"

#include <ecrt.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <ctime>
#include <map>
#include <pthread.h>
#include <sched.h>
#include <thread>
#include <utility>
#include <unistd.h>
#include <vector>

namespace RmdCanSdk {

struct EcIoctlSlaveState {
    unsigned short slave_position;
    unsigned char al_state;
};

#define EC_IOCTL_TYPE 0xa4
#define EC_IOW(nr, type) _IOW(EC_IOCTL_TYPE, nr, type)
#define EC_IOCTL_SLAVE_STATE EC_IOW(0x0b, EcIoctlSlaveState)

struct EthercatDomainRuntime {
    int domainId = 0;
    int division = 1;
    ec_domain_t* domain = nullptr;
    std::uint8_t* data = nullptr;
    std::size_t size = 0;
    ec_domain_state_t state{};
    unsigned int lastWorkingCounter = 0xffffffffu;
    ec_wc_state_t lastWcState = static_cast<ec_wc_state_t>(0xff);
    std::vector<EthercatPdoBinding> bindings;
};

class RmdEthercatRuntime {
public:
    RmdEthercatRuntime(Config const& c,
                       MotorRegistry const& r,
                       std::size_t b,
                       FrameBuffer<EthercatPackedTargetFrame>& tb,
                       FrameBuffer<MotorActualFrame>& ab,
                       std::vector<char> modes)
        : config(c), registry(r), backendIndex(b), targetBuffer(tb), actualBuffer(ab), operatingModes(std::move(modes)) {}

    Config const& config;
    MotorRegistry const& registry;
    std::size_t backendIndex;
    FrameBuffer<EthercatPackedTargetFrame>& targetBuffer;
    FrameBuffer<MotorActualFrame>& actualBuffer;
    std::vector<char> operatingModes;
    AtomicBackendStatus status;
    std::atomic<bool> stop{true};
    std::thread thread;
    MotorActualFrame workingActuals;
    EthercatPackedTargetFrame targets;
    ec_master_t* master = nullptr;
    int masterOrder = 0;
    int masterFd = -1;
    unsigned int slaveCount = 0;
    std::vector<EthercatDomainRuntime> domains;
    int lastSlavesResponding = -1;
    unsigned int lastAlStates = 0xffffffffu;
};

namespace {

int failStart(RmdEthercatRuntime* rt) {
    rt->status.setRunning(false);
    rt->status.setFault(ErrorCodeBackendFault);
    if (rt->master != nullptr) {
        ecrt_release_master(rt->master);
        rt->master = nullptr;
    }
    if (rt->masterFd >= 0) {
        close(rt->masterFd);
        rt->masterFd = -1;
    }
    rt->domains.clear();
    return -1;
}

ec_pdo_entry_info_t toEcrtEntry(EthercatPdoEntrySpec const& entry) {
    return ec_pdo_entry_info_t{entry.index, entry.subindex, entry.bitLength};
}

long periodNs(Config const& config) {
    return config.periodNs > 0 ? config.periodNs : 1000000L;
}

std::uint64_t timespecToNs(timespec const& value) {
    return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ull + static_cast<std::uint64_t>(value.tv_nsec);
}

int domainDivision(Config const& config, int domainId) {
    if (domainId >= 0) {
        std::size_t const index = static_cast<std::size_t>(domainId);
        if (index < config.ethercatDomainDivisions.size()) {
            return std::max(1, config.ethercatDomainDivisions[index]);
        }
    }
    return 1;
}

void addNs(timespec& time, long ns) {
    constexpr long NsecPerSec = 1000000000L;
    time.tv_nsec += ns;
    while (time.tv_nsec >= NsecPerSec) {
        time.tv_nsec -= NsecPerSec;
        time.tv_sec++;
    }
}

void sleepUntil(timespec const& wakeup) {
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, nullptr) == EINTR) {
    }
}

bool requestSlaveState(int masterFd, int slave, unsigned char state) {
    EcIoctlSlaveState data{};
    data.slave_position = static_cast<unsigned short>(slave);
    data.al_state = state;
    if (ioctl(masterFd, EC_IOCTL_SLAVE_STATE, &data) < 0) {
        std::fprintf(stderr, "requesting EtherCAT slave %d state 0x%02x failed: errno=%d\n", slave, state, errno);
        return false;
    }
    return true;
}

int envInt(char const* name, int fallback) {
    char const* value = std::getenv(name);
    if (value == nullptr || *value == '\0') {
        return fallback;
    }
    char* end = nullptr;
    long const parsed = std::strtol(value, &end, 10);
    if (end == value) {
        return fallback;
    }
    return static_cast<int>(parsed);
}

int lastOnlineCpu() {
    long const count = sysconf(_SC_NPROCESSORS_ONLN);
    return count > 0 ? static_cast<int>(count - 1) : 0;
}

void lockRealtimeMemory() {
    if (envInt("RMD_ECAT_RT_MLOCK", 1) == 0) {
        return;
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::fprintf(stderr, "mlockall(MCL_CURRENT|MCL_FUTURE) failed: errno=%d\n", errno);
    }
}

void configureRealtimeThread() {
    pthread_setname_np(pthread_self(), "rmd_ecat_rt");

    int const cpu = envInt("RMD_ECAT_RT_CPU", lastOnlineCpu());
    if (cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cpu, &set);
        int const rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
        if (rc != 0) {
            std::fprintf(stderr, "setting EtherCAT RT thread affinity to CPU %d failed: errno=%d\n", cpu, rc);
        }
    }

    int const priority = envInt("RMD_ECAT_RT_PRIORITY", 80);
    if (priority > 0) {
        sched_param param{};
        param.sched_priority = priority;
        int const rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (rc != 0) {
            std::fprintf(stderr, "setting EtherCAT RT thread SCHED_FIFO priority %d failed: errno=%d\n", priority, rc);
        }
    }

    if (prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL) != 0) {
        std::fprintf(stderr, "setting EtherCAT RT thread timer slack failed: errno=%d\n", errno);
    }
}

bool stateLogEnabled() {
    static bool const enabled = [] {
        char const* value = std::getenv("RMD_ECAT_LOG_STATE");
        if (value == nullptr) {
            return false;
        }
        return std::atoi(value) != 0;
    }();
    return enabled;
}

void requestSlaveRangeState(int masterFd, unsigned int slaveCount, unsigned char state) {
    for (unsigned int slave = 0; slave < slaveCount; ++slave) {
        while (!requestSlaveState(masterFd, static_cast<int>(slave), state)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

void requestAllSlavesState(RmdEthercatRuntime* rt, unsigned char state) {
    if (rt->masterFd < 0) {
        return;
    }
    if (rt->master == nullptr) {
        requestSlaveRangeState(rt->masterFd, rt->slaveCount, state);
        return;
    }
    ec_master_info_t masterInfo{};
    if (ecrt_master(rt->master, &masterInfo) == 0) {
        rt->slaveCount = masterInfo.slave_count;
        requestSlaveRangeState(rt->masterFd, rt->slaveCount, state);
        return;
    }
    for (EthercatDomainRuntime const& domain : rt->domains) {
        for (EthercatPdoBinding const& binding : domain.bindings) {
            while (!requestSlaveState(rt->masterFd, resolveEthercatSlavePosition(binding), state)) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
}

bool downloadSdo(ec_master_t* master,
                 int slave,
                 std::uint16_t index,
                 std::uint8_t subindex,
                 void const* data,
                 std::size_t size,
                 char const* label) {
    unsigned int abortCode = 0;
    int const rc = ecrt_master_sdo_download(master,
                                            slave,
                                            index,
                                            subindex,
                                            static_cast<unsigned char*>(const_cast<void*>(data)),
                                            size,
                                            &abortCode);
    if (rc < 0) {
        std::fprintf(stderr,
                     "EtherCAT slave %d SDO download %s 0x%04x:%02x failed, abortCode=0x%08x\n",
                     slave,
                     label,
                     index,
                     subindex,
                     abortCode);
        return false;
    }
    return true;
}

bool remapMtDevicePdos(ec_master_t* master,
                       int slave,
                       EthercatMtDevicePdoSpec const& spec,
                       EthercatMtDevicePdoProfileSpec const& rxSpec,
                       EthercatMtDevicePdoProfileSpec const& txSpec) {
    unsigned char u8 = 0;
    unsigned short u16 = 0;

    u8 = 0;
    if (!downloadSdo(master, slave, spec.rxAssignmentIndex, 0x00, &u8, sizeof(u8), "clear RxPDO assignment")) {
        return false;
    }
    u16 = rxSpec.pdoIndex;
    if (!downloadSdo(master, slave, spec.rxAssignmentIndex, 0x01, &u16, sizeof(u16), "set RxPDO assignment")) {
        return false;
    }
    u8 = 1;
    if (!downloadSdo(master, slave, spec.rxAssignmentIndex, 0x00, &u8, sizeof(u8), "enable RxPDO assignment")) {
        return false;
    }

    u8 = 0;
    if (!downloadSdo(master, slave, spec.txAssignmentIndex, 0x00, &u8, sizeof(u8), "clear TxPDO assignment")) {
        return false;
    }
    u16 = txSpec.pdoIndex;
    if (!downloadSdo(master, slave, spec.txAssignmentIndex, 0x01, &u16, sizeof(u16), "set TxPDO assignment")) {
        return false;
    }
    u8 = 1;
    return downloadSdo(master, slave, spec.txAssignmentIndex, 0x00, &u8, sizeof(u8), "enable TxPDO assignment");
}

bool configureMtDevicePdos(RmdEthercatRuntime* rt,
                           EthercatDomainRuntime& domain,
                           EthercatPdoBinding& binding) {
    if (!isEthercatMtDeviceType(binding.type)) {
        return false;
    }

    EthercatMtDevicePdoSpec const& spec = ethercatMtDevicePdoSpec();
    EthercatMtDevicePdoProfileSpec const& rxSpec = ethercatMtDeviceRxPdoSpec(binding.rxProfile);
    EthercatMtDevicePdoProfileSpec const& txSpec = ethercatMtDeviceTxPdoSpec(binding.txProfile);
    int const slave = resolveEthercatSlavePosition(binding);
    if (rt->masterFd >= 0) {
        while (!requestSlaveState(rt->masterFd, slave, 0x02)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    if (!remapMtDevicePdos(rt->master, slave, spec, rxSpec, txSpec)) {
        return false;
    }

    ec_slave_config_t* slaveConfig = ecrt_master_slave_config(rt->master, 0, slave, spec.vendorId, spec.productCode);
    if (slaveConfig == nullptr) {
        return false;
    }

    std::vector<ec_pdo_entry_info_t> entries;
    entries.reserve(rxSpec.entries.size() + txSpec.entries.size());
    for (EthercatPdoEntrySpec const& entry : rxSpec.entries) {
        entries.push_back(toEcrtEntry(entry));
    }
    for (EthercatPdoEntrySpec const& entry : txSpec.entries) {
        entries.push_back(toEcrtEntry(entry));
    }

    std::array<ec_pdo_info_t, 2> pdoInfos{{
        {rxSpec.pdoIndex, static_cast<unsigned int>(rxSpec.entries.size()), entries.data()},
        {txSpec.pdoIndex, static_cast<unsigned int>(txSpec.entries.size()), entries.data() + rxSpec.entries.size()},
    }};

    std::array<ec_sync_info_t, 5> syncInfos{{
        {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, pdoInfos.data(), EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, pdoInfos.data() + 1, EC_WD_DISABLE},
        {0xff},
    }};

    if (ecrt_slave_config_pdos(slaveConfig, EC_END, syncInfos.data()) < 0) {
        return false;
    }

    if (rt->config.ethercatDc) {
        long const sync0Cycle = periodNs(rt->config) * domain.division;
        ecrt_slave_config_dc(slaveConfig, 0x0300, sync0Cycle, sync0Cycle / 2, 0, 0);
    }

    unsigned int bitPosition = 0;
    int const rxOffset =
        ecrt_slave_config_reg_pdo_entry(slaveConfig, entries[0].index, entries[0].subindex, domain.domain, &bitPosition);
    if (rxOffset < 0) {
        return false;
    }
    for (std::size_t i = 1; i < rxSpec.entries.size(); ++i) {
        if (ecrt_slave_config_reg_pdo_entry(
                slaveConfig, entries[i].index, entries[i].subindex, domain.domain, &bitPosition) < 0) {
            return false;
        }
    }

    std::size_t const txBegin = rxSpec.entries.size();
    int const txOffset = ecrt_slave_config_reg_pdo_entry(
        slaveConfig, entries[txBegin].index, entries[txBegin].subindex, domain.domain, &bitPosition);
    if (txOffset < 0) {
        return false;
    }
    for (std::size_t i = txBegin + 1; i < entries.size(); ++i) {
        if (ecrt_slave_config_reg_pdo_entry(
                slaveConfig, entries[i].index, entries[i].subindex, domain.domain, &bitPosition) < 0) {
            return false;
        }
    }

    binding.slave = slave;
    binding.rxOffset = rxOffset;
    binding.txOffset = txOffset;
    binding.rxSize = ethercatMtDeviceRxPdoSize(binding.rxProfile);
    binding.txSize = ethercatMtDeviceTxPdoSize(binding.txProfile);
    return true;
}

void realtimeLoop(RmdEthercatRuntime* rt) {
    configureRealtimeThread();

    timespec wakeup{};
    clock_gettime(CLOCK_MONOTONIC, &wakeup);
    long const cycleNs = periodNs(rt->config);

    while (!rt->stop.load(std::memory_order_acquire)) {
        addNs(wakeup, cycleNs);
        sleepUntil(wakeup);
        auto const started = std::chrono::steady_clock::now();

        ecrt_master_receive(rt->master);
        ec_master_state_t masterState{};
        ecrt_master_state(rt->master, &masterState);
        if (rt->lastSlavesResponding != static_cast<int>(masterState.slaves_responding)) {
            rt->lastSlavesResponding = static_cast<int>(masterState.slaves_responding);
            if (stateLogEnabled()) {
                std::fprintf(stderr,
                             "EtherCAT master %d slaves_responding changed to %d\n",
                             rt->masterOrder,
                             rt->lastSlavesResponding);
            }
        }
        if (rt->lastAlStates != masterState.al_states) {
            rt->lastAlStates = masterState.al_states;
            if (stateLogEnabled()) {
                std::fprintf(stderr,
                             "EtherCAT master %d al_states changed to 0x%02x\n",
                             rt->masterOrder,
                             rt->lastAlStates);
            }
        }

        rt->workingActuals.sequence++;
        rt->workingActuals.timestamp = RealtimeClock::now();
        rt->workingActuals.motorCount = static_cast<std::size_t>(rt->config.totalMotorCount);
        for (EthercatDomainRuntime& domain : rt->domains) {
            ecrt_domain_process(domain.domain);
            ecrt_domain_state(domain.domain, &domain.state);
            if (domain.lastWorkingCounter != domain.state.working_counter) {
                domain.lastWorkingCounter = domain.state.working_counter;
                if (stateLogEnabled()) {
                    std::fprintf(stderr,
                                 "EtherCAT master %d domain %d working_counter changed to %u\n",
                                 rt->masterOrder,
                                 domain.domainId,
                                 domain.state.working_counter);
                }
            }
            if (domain.lastWcState != domain.state.wc_state) {
                domain.lastWcState = domain.state.wc_state;
                if (stateLogEnabled()) {
                    std::fprintf(stderr,
                                 "EtherCAT master %d domain %d wc_state changed to %u\n",
                                 rt->masterOrder,
                                 domain.domainId,
                                 static_cast<unsigned int>(domain.state.wc_state));
                }
            }
            if (domain.state.wc_state == EC_WC_COMPLETE) {
                readEthercatDomainSnapshot(domain.data, domain.size, domain.bindings, rt->workingActuals);
            } else {
                readEthercatDomainSnapshot(nullptr, 0, domain.bindings, rt->workingActuals);
                rt->status.recordWcIncomplete();
                rt->status.recordStaleFrame();
                rt->status.setFault(ErrorCodeFeedbackTimeout);
            }
        }

        if (rt->config.ethercatDc) {
            timespec currentTime{};
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            std::uint64_t const appTime = timespecToNs(currentTime);
            ecrt_master_application_time(rt->master, appTime);
            ecrt_master_sync_reference_clock_to(rt->master, appTime);
            ecrt_master_sync_slave_clocks(rt->master);
        }

        rt->targetBuffer.readInto(rt->targets);
        for (EthercatDomainRuntime& domain : rt->domains) {
            writeEthercatDomainSnapshot(domain.data, domain.size, domain.bindings, rt->targets, rt->workingActuals);
            ecrt_domain_queue(domain.domain);
        }

        rt->actualBuffer.publish(rt->workingActuals);
        ecrt_master_send(rt->master);

        auto const finished = std::chrono::steady_clock::now();
        auto const elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(finished - started);
        rt->status.recordCycle(static_cast<std::uint64_t>(elapsed.count()), static_cast<std::uint64_t>(cycleNs));
    }
}

} // namespace

RmdEthercatBackend::RmdEthercatBackend(Config const& config,
                                       MotorRegistry const& registry,
                                       std::size_t backendIndex,
                                       FrameBuffer<EthercatPackedTargetFrame>& targetBuffer,
                                       FrameBuffer<MotorActualFrame>& actualBuffer,
                                       std::vector<char> operatingModes)
    : runtime_(new RmdEthercatRuntime(
          config, registry, backendIndex, targetBuffer, actualBuffer, std::move(operatingModes))) {}

RmdEthercatBackend::~RmdEthercatBackend() {
    stop();
    delete runtime_;
}

int RmdEthercatBackend::start() {
    auto const& backend = runtime_->registry.backends().at(runtime_->backendIndex);
    std::vector<EthercatPdoBinding> bindings = buildEthercatPdoBindings(runtime_->registry, runtime_->backendIndex);
    assignEthercatPdoProfiles(bindings, runtime_->operatingModes);
    runtime_->masterOrder = backend.master;
    runtime_->master = ecrt_request_master(backend.master);
    if (runtime_->master == nullptr) {
        return failStart(runtime_);
    }
    char deviceName[64]{};
    std::snprintf(deviceName, sizeof(deviceName), "/dev/EtherCAT%d", backend.master);
    runtime_->masterFd = open(deviceName, O_RDWR);
    if (runtime_->masterFd < 0) {
        std::fprintf(stderr, "opening EtherCAT master device %s failed: errno=%d\n", deviceName, errno);
        return failStart(runtime_);
    }
    ec_master_info_t masterInfo{};
    if (ecrt_master(runtime_->master, &masterInfo) == 0) {
        runtime_->slaveCount = masterInfo.slave_count;
    }

    std::map<int, std::size_t> domainById;
    for (EthercatPdoBinding const& binding : bindings) {
        auto it = domainById.find(binding.domain);
        if (it == domainById.end()) {
            EthercatDomainRuntime domain;
            domain.domainId = binding.domain;
            domain.division = domainDivision(runtime_->config, binding.domain);
            domain.domain = ecrt_master_create_domain(runtime_->master);
            if (domain.domain == nullptr) {
                return failStart(runtime_);
            }
            runtime_->domains.push_back(domain);
            it = domainById.emplace(binding.domain, runtime_->domains.size() - 1).first;
        }
        runtime_->domains[it->second].bindings.push_back(binding);
    }

    for (EthercatDomainRuntime& domain : runtime_->domains) {
        for (EthercatPdoBinding& binding : domain.bindings) {
            if (!configureMtDevicePdos(runtime_, domain, binding)) {
                return failStart(runtime_);
            }
        }
    }

    if (ecrt_master_activate(runtime_->master) != 0) {
        return failStart(runtime_);
    }

    lockRealtimeMemory();

    for (EthercatDomainRuntime& domain : runtime_->domains) {
        domain.data = ecrt_domain_data(domain.domain);
        if (domain.data == nullptr) {
            return failStart(runtime_);
        }
        domain.size = ecrt_domain_size(domain.domain);
    }

    runtime_->stop.store(false, std::memory_order_release);
    runtime_->status.setRunning(true);
    runtime_->thread = std::thread(realtimeLoop, runtime_);
    requestAllSlavesState(runtime_, 0x08);
    return 0;
}

void RmdEthercatBackend::stop() {
    if (runtime_ == nullptr) {
        return;
    }
    runtime_->stop.store(true, std::memory_order_release);
    if (runtime_->thread.joinable()) {
        runtime_->thread.join();
    }
    if (runtime_->masterFd >= 0 && runtime_->slaveCount > 0) {
        requestSlaveRangeState(runtime_->masterFd, runtime_->slaveCount, 0x01);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        requestSlaveRangeState(runtime_->masterFd, runtime_->slaveCount, 0x02);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (runtime_->master != nullptr) {
        ecrt_release_master(runtime_->master);
        runtime_->master = nullptr;
    }
    if (runtime_->masterFd >= 0 && runtime_->slaveCount > 0) {
        requestSlaveRangeState(runtime_->masterFd, runtime_->slaveCount, 0x01);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        requestSlaveRangeState(runtime_->masterFd, runtime_->slaveCount, 0x02);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (runtime_->masterFd >= 0) {
        close(runtime_->masterFd);
        runtime_->masterFd = -1;
    }
    runtime_->domains.clear();
    runtime_->status.setRunning(false);
}

BackendStatus RmdEthercatBackend::status() const {
    return runtime_ == nullptr ? BackendStatus{} : runtime_->status.snapshot();
}

} // namespace RmdCanSdk
