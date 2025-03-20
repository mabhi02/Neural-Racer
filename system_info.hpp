#include "neural_racer/hardware/system_info.hpp"
#include <iostream>
#include <sstream>
#include <memory>
#include <stdexcept>
#include <vector>
#include <string>
#include <algorithm>

#ifdef _WIN32
#include <windows.h>
#include <sysinfoapi.h>
#elif __linux__
#include <unistd.h>
#include <sys/sysinfo.h>
#include <sys/utsname.h>
#include <fstream>
#elif __APPLE__
#include <sys/types.h>
#include <sys/sysctl.h>
#include <mach/mach.h>
#endif

namespace neural_racer {
namespace hardware {

std::string SystemInfo::getOSName() {
#ifdef _WIN32
    return "Windows";
#elif __APPLE__
    return "macOS";
#elif __linux__
    return "Linux";
#else
    return "Unknown";
#endif
}

std::string SystemInfo::getOSVersion() {
#ifdef _WIN32
    // Windows version information
    OSVERSIONINFOEX osvi;
    ZeroMemory(&osvi, sizeof(OSVERSIONINFOEX));
    osvi.dwOSVersionInfoSize = sizeof(OSVERSIONINFOEX);
    
    // Note: GetVersionEx is deprecated, but we're using it for simplicity
    // A production implementation should use RtlGetVersion or VerifyVersionInfo
    if (GetVersionEx((OSVERSIONINFO*)&osvi)) {
        std::stringstream ss;
        ss << osvi.dwMajorVersion << "." << osvi.dwMinorVersion;
        if (osvi.wServicePackMajor > 0) {
            ss << " SP" << osvi.wServicePackMajor;
        }
        return ss.str();
    }
    return "Unknown Windows Version";
#elif __linux__
    // Linux version from uname
    struct utsname buf;
    if (uname(&buf) == 0) {
        return std::string(buf.release);
    }
    
    // Fallback to /proc/version
    std::ifstream version("/proc/version");
    if (version.is_open()) {
        std::string line;
        std::getline(version, line);
        version.close();
        
        // Extract version from the line
        size_t pos = line.find("version ");
        if (pos != std::string::npos) {
            std::string versionStr = line.substr(pos + 8);
            pos = versionStr.find(' ');
            if (pos != std::string::npos) {
                return versionStr.substr(0, pos);
            }
            return versionStr;
        }
    }
    return "Unknown Linux Version";
#elif __APPLE__
    // macOS version via sysctl
    char str[256];
    size_t size = sizeof(str);
    int ret = sysctlbyname("kern.osrelease", str, &size, NULL, 0);
    if (ret == 0) {
        return std::string(str, size);
    }
    return "Unknown macOS Version";
#else
    return "Unknown OS Version";
#endif
}

int SystemInfo::getProcessorCoreCount() {
#ifdef _WIN32
    SYSTEM_INFO sysInfo;
    GetSystemInfo(&sysInfo);
    return sysInfo.dwNumberOfProcessors;
#elif __APPLE__
    int cores;
    size_t len = sizeof(cores);
    sysctlbyname("hw.logicalcpu", &cores, &len, NULL, 0);
    return cores;
#else
    // Linux and others use sysconf
    return sysconf(_SC_NPROCESSORS_ONLN);
#endif
}

uint64_t SystemInfo::getTotalSystemMemory() {
#ifdef _WIN32
    MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memInfo);
    return memInfo.ullTotalPhys;
#elif __APPLE__
    int64_t physical_memory;
    int mib[2];
    size_t length;

    // Get the Physical memory size
    mib[0] = CTL_HW;
    mib[1] = HW_MEMSIZE;
    length = sizeof(physical_memory);
    if (sysctl(mib, 2, &physical_memory, &length, NULL, 0) == 0) {
        return physical_memory;
    }
    return 0;
#elif __linux__
    struct sysinfo memInfo;
    sysinfo(&memInfo);
    return memInfo.totalram * memInfo.mem_unit;
#else
    return 0;
#endif
}

uint64_t SystemInfo::getAvailableSystemMemory() {
#ifdef _WIN32
    MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memInfo);
    return memInfo.ullAvailPhys;
#elif __APPLE__
    vm_statistics_data_t vm_stats;
    mach_port_t host_port = mach_host_self();
    mach_msg_type_number_t host_size = sizeof(vm_statistics_data_t) / sizeof(integer_t);
    
    if (host_statistics(host_port, HOST_VM_INFO, (host_info_t)&vm_stats, &host_size) == KERN_SUCCESS) {
        return (uint64_t)vm_stats.free_count * 4096; // page size is 4096 bytes
    }
    return 0;
#elif __linux__
    struct sysinfo memInfo;
    sysinfo(&memInfo);
    return memInfo.freeram * memInfo.mem_unit;
#else
    return 0;
#endif
}

std::vector<std::string> SystemInfo::getGPUInfo() {
    std::vector<std::string> gpuInfo;
    
#ifdef _WIN32
    // Windows GPU detection requires WMI or DXGI
    // For a simplified version, we'll use a simulation approach
    // A real implementation would use WMI or the DXGI API
    
    // This is just a placeholder for the concept
    gpuInfo.push_back("GPU detection requires WMI or DXGI API (simulated)");
    
    // Check if a dedicated GPU might be present
    MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memInfo);
    
    // Heuristic: Systems with 8GB+ RAM often have dedicated GPUs
    if (memInfo.ullTotalPhys > 8ULL * 1024 * 1024 * 1024) {
        gpuInfo.push_back("Potential dedicated GPU detected (8GB+ system)");
    }
#elif __linux__
    // Check for NVIDIA GPUs through the /proc filesystem
    std::ifstream nvidiaProc("/proc/driver/nvidia/gpus/0/information");
    if (nvidiaProc.good()) {
        std::string line;
        if (std::getline(nvidiaProc, line)) {
            gpuInfo.push_back("NVIDIA GPU: " + line);
        } else {
            gpuInfo.push_back("NVIDIA GPU detected");
        }
        nvidiaProc.close();
    }
    
    // Check for AMD GPUs through device enumeration
    std::ifstream amdProc("/sys/class/drm/card0/device/vendor");
    if (amdProc.good()) {
        std::string vendor;
        amdProc >> vendor;
        if (vendor == "0x1002") {  // AMD vendor ID
            gpuInfo.push_back("AMD GPU detected");
        }
        amdProc.close();
    }
    
    // If no GPU was detected through specific drivers, check generic info
    if (gpuInfo.empty()) {
        // Check lspci output indirectly through a simplified approach
        std::ifstream lspciOutput("/tmp/lspci_output.txt");
        if (lspciOutput.good()) {
            std::string line;
            while (std::getline(lspciOutput, line)) {
                if (line.find("VGA compatible controller") != std::string::npos ||
                    line.find("3D controller") != std::string::npos) {
                    gpuInfo.push_back(line);
                }
            }
            lspciOutput.close();
        }
    }
#elif __APPLE__
    // macOS GPU detection
    char buf[100];
    size_t buflen = sizeof(buf);
    if (sysctlbyname("machdep.cpu.brand_string", &buf, &buflen, NULL, 0) == 0) {
        std::string cpuInfo(buf);
        
        // Check for integrated GPU based on CPU
        if (cpuInfo.find("Intel") != std::string::npos) {
            gpuInfo.push_back("Intel Integrated GPU (likely)");
        } else if (cpuInfo.find("Apple") != std::string::npos) {
            gpuInfo.push_back("Apple Silicon Integrated GPU (likely)");
        }
        
        // Check for dedicated GPU
        // In a real implementation, IOKit would be used to enumerate GPUs
        // This is a simplified simulation that checks common GPU vendors
        std::ifstream ioregOutput("/tmp/ioreg_output.txt");
        if (ioregOutput.good()) {
            std::string line;
            while (std::getline(ioregOutput, line)) {
                if (line.find("AMD") != std::string::npos && line.find("controller") != std::string::npos) {
                    gpuInfo.push_back("AMD Dedicated GPU detected");
                } else if (line.find("NVIDIA") != std::string::npos && line.find("controller") != std::string::npos) {
                    gpuInfo.push_back("NVIDIA Dedicated GPU detected");
                }
            }
            ioregOutput.close();
        }
    }
#endif

    // If no GPU was detected, add a generic entry
    if (gpuInfo.empty()) {
        gpuInfo.push_back("Generic GPU (detected through simulation)");
    }
    
    return gpuInfo;
}

bool SystemInfo::hasCUDASupport() {
    // In a real implementation, this would check for CUDA runtime or driver
    // Here we use a simplified simulation approach
    
#ifdef _WIN32
    // Check if a CUDA-related DLL might exist
    HMODULE cudaModule = LoadLibraryA("nvcuda.dll");
    if (cudaModule != NULL) {
        FreeLibrary(cudaModule);
        return true;
    }
    return false;
#elif __linux__
    // Check for NVIDIA driver presence
    std::ifstream nvidiaProc("/proc/driver/nvidia/version");
    if (nvidiaProc.good()) {
        nvidiaProc.close();
        return true;
    }
    
    // Check if CUDA runtime might be installed
    std::ifstream cudaLib("/usr/local/cuda/lib64/libcudart.so");
    if (cudaLib.good()) {
        cudaLib.close();
        return true;
    }
    
    return false;
#elif __APPLE__
    // CUDA support was removed from macOS after 10.13 High Sierra
    // Check for NVIDIA drivers in common location
    std::ifstream nvidiaDriver("/Library/Extensions/NVDAGK100Hal.kext");
    if (nvidiaDriver.good()) {
        nvidiaDriver.close();
        return true;
    }
    
    return false;
#else
    return false;
#endif
}

bool SystemInfo::hasOpenCLSupport() {
    // In a real implementation, this would check for OpenCL runtime
    // Here we use a simplified simulation approach
    
#ifdef _WIN32
    // Check if an OpenCL-related DLL might exist
    HMODULE openclModule = LoadLibraryA("OpenCL.dll");
    if (openclModule != NULL) {
        FreeLibrary(openclModule);
        return true;
    }
    return false;
#elif __linux__
    // Check for OpenCL ICD loader
    std::ifstream openclLib("/usr/lib/libOpenCL.so");
    if (openclLib.good()) {
        openclLib.close();
        return true;
    }
    
    // Alternative location
    std::ifstream openclLib2("/usr/lib/x86_64-linux-gnu/libOpenCL.so");
    if (openclLib2.good()) {
        openclLib2.close();
        return true;
    }
    
    return false;
#elif __APPLE__
    // OpenCL is available on all macOS versions (up to a point)
    // In a real implementation, we would check the OS version
    // For simulation, we'll assume it's available
    return true;
#else
    return false;
#endif
}

} // namespace hardware
} // namespace neural_racer