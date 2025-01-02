/**
 * @file tensorrt.cpp
 * @brief TensorRT utility library for DeepStream pipeline optimization
 * @version 1.0
 * 
 * Provides core functionality for model loading, optimization, and inference
 * execution with CUDA acceleration for the DeepStream pipeline.
 */

#include <NvInfer.h>      // TensorRT 8.5
#include <cuda_runtime.h> // CUDA 11.4
#include <nvonnxparser.h> // TensorRT 8.5

#include <fstream>
#include <memory>
#include <mutex>
#include <chrono>
#include <vector>
#include <stdexcept>
#include <filesystem>

// Global constants for TensorRT configuration
constexpr size_t WORKSPACE_SIZE = 1ULL << 30;        // 1GB workspace
constexpr int MAX_BATCH_SIZE = 8;                    // Maximum batch size for inference
constexpr bool FP16_MODE = true;                     // Enable FP16 precision by default
constexpr const char* ENGINE_CACHE_VERSION = "v1.0"; // Engine cache version
constexpr int DEFAULT_MIN_TIMING_ITERATIONS = 3;     // Minimum timing iterations

/**
 * @class TensorRTLogger
 * @brief Thread-safe logging implementation for TensorRT operations
 */
class TensorRTLogger : public nvinfer1::ILogger {
private:
    Severity minSeverity;
    std::mutex logMutex;
    std::ofstream logFile;
    
    std::string getSeverityString(Severity severity) const {
        switch (severity) {
            case Severity::kINTERNAL_ERROR: return "INTERNAL_ERROR";
            case Severity::kERROR: return "ERROR";
            case Severity::kWARNING: return "WARNING";
            case Severity::kINFO: return "INFO";
            case Severity::kVERBOSE: return "VERBOSE";
            default: return "UNKNOWN";
        }
    }

public:
    TensorRTLogger(Severity severity, const std::string& logPath) 
        : minSeverity(severity) {
        logFile.open(logPath, std::ios::out | std::ios::app);
        if (!logFile.is_open()) {
            throw std::runtime_error("Failed to open TensorRT log file: " + logPath);
        }
    }

    ~TensorRTLogger() {
        if (logFile.is_open()) {
            logFile.close();
        }
    }

    void log(Severity severity, const char* msg) noexcept override {
        if (severity > minSeverity) return;

        std::lock_guard<std::mutex> lock(logMutex);
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::system_clock::to_time_t(now);

        logFile << "[" << std::ctime(&timestamp) << "][" 
                << getSeverityString(severity) << "] " 
                << msg << std::endl;

        if (severity <= Severity::kERROR) {
            logFile.flush();
        }
    }
};

/**
 * @class TensorRTOptimizationConfig
 * @brief Configuration class for TensorRT engine optimization
 */
class TensorRTOptimizationConfig {
public:
    size_t maxWorkspaceSize;
    int maxBatchSize;
    bool fp16Enabled;
    bool int8Enabled;
    std::vector<nvinfer1::Dims> optimizationProfiles;
    int minTimingIterations;
    bool strictTypeConstraints;

    TensorRTOptimizationConfig()
        : maxWorkspaceSize(WORKSPACE_SIZE)
        , maxBatchSize(MAX_BATCH_SIZE)
        , fp16Enabled(FP16_MODE)
        , int8Enabled(false)
        , minTimingIterations(DEFAULT_MIN_TIMING_ITERATIONS)
        , strictTypeConstraints(true) {}

    bool setOptimizationProfile(
        const nvinfer1::Dims& minDims,
        const nvinfer1::Dims& optDims,
        const nvinfer1::Dims& maxDims
    ) {
        // Validate dimensions
        if (minDims.nbDims != optDims.nbDims || optDims.nbDims != maxDims.nbDims) {
            return false;
        }

        for (int i = 0; i < minDims.nbDims; i++) {
            if (minDims.d[i] > optDims.d[i] || optDims.d[i] > maxDims.d[i]) {
                return false;
            }
        }

        optimizationProfiles.push_back(minDims);
        optimizationProfiles.push_back(optDims);
        optimizationProfiles.push_back(maxDims);
        return true;
    }
};

/**
 * @brief Creates and configures a TensorRT inference engine
 * @param modelPath Path to the ONNX model file
 * @param config Engine optimization configuration
 * @param enableProfiling Enable engine profiling
 * @return Optimized TensorRT engine pointer
 */
[[nodiscard]]
nvinfer1::ICudaEngine* createInferenceEngine(
    const char* modelPath,
    TensorRTOptimizationConfig& config,
    bool enableProfiling
) {
    // Validate inputs
    if (!std::filesystem::exists(modelPath)) {
        throw std::runtime_error("Model file not found: " + std::string(modelPath));
    }

    // Initialize CUDA context
    cudaSetDevice(0);
    
    // Create TensorRT builder and network
    auto logger = std::make_unique<TensorRTLogger>(
        nvinfer1::ILogger::Severity::kWARNING,
        "tensorrt.log"
    );
    
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(
        nvinfer1::createInferBuilder(*logger)
    );
    if (!builder) {
        throw std::runtime_error("Failed to create TensorRT builder");
    }

    const auto explicitBatch = 1U << static_cast<uint32_t>(
        nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH
    );
    
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
        builder->createNetworkV2(explicitBatch)
    );
    if (!network) {
        throw std::runtime_error("Failed to create network definition");
    }

    // Create ONNX parser
    auto parser = std::unique_ptr<nvonnxparser::IParser>(
        nvonnxparser::createParser(*network, *logger)
    );
    if (!parser) {
        throw std::runtime_error("Failed to create ONNX parser");
    }

    // Parse ONNX model
    if (!parser->parseFromFile(modelPath, static_cast<int>(
        nvinfer1::ILogger::Severity::kWARNING))) {
        throw std::runtime_error("Failed to parse ONNX model");
    }

    // Configure builder
    auto config_flags = std::unique_ptr<nvinfer1::IBuilderConfig>(
        builder->createBuilderConfig()
    );
    if (!config_flags) {
        throw std::runtime_error("Failed to create builder configuration");
    }

    config_flags->setMaxWorkspaceSize(config.maxWorkspaceSize);
    
    if (config.fp16Enabled && builder->platformHasFastFp16()) {
        config_flags->setFlag(nvinfer1::BuilderFlag::kFP16);
    }

    if (config.int8Enabled && builder->platformHasFastInt8()) {
        config_flags->setFlag(nvinfer1::BuilderFlag::kINT8);
    }

    if (enableProfiling) {
        config_flags->setProfileStream(cudaStreamDefault);
    }

    // Set optimization profiles
    auto profile = builder->createOptimizationProfile();
    for (size_t i = 0; i < config.optimizationProfiles.size(); i += 3) {
        profile->setDimensions(
            network->getInput(0)->getName(),
            nvinfer1::OptProfileSelector::kMIN,
            config.optimizationProfiles[i]
        );
        profile->setDimensions(
            network->getInput(0)->getName(),
            nvinfer1::OptProfileSelector::kOPT,
            config.optimizationProfiles[i + 1]
        );
        profile->setDimensions(
            network->getInput(0)->getName(),
            nvinfer1::OptProfileSelector::kMAX,
            config.optimizationProfiles[i + 2]
        );
    }
    config_flags->addOptimizationProfile(profile);

    // Build engine
    auto engine = std::unique_ptr<nvinfer1::ICudaEngine>(
        builder->buildEngineWithConfig(*network, *config_flags)
    );
    if (!engine) {
        throw std::runtime_error("Failed to build TensorRT engine");
    }

    return engine.release();
}

/**
 * @brief Serializes a TensorRT engine to disk
 * @param engine TensorRT engine to serialize
 * @param enginePath Path to save the serialized engine
 * @param enableCompression Enable engine compression
 * @return Success status
 */
bool serializeEngine(
    nvinfer1::ICudaEngine* engine,
    const char* enginePath,
    bool enableCompression
) {
    if (!engine || !enginePath) {
        return false;
    }

    // Create serialization stream
    auto serializedEngine = std::unique_ptr<nvinfer1::IHostMemory>(
        engine->serialize()
    );
    if (!serializedEngine) {
        return false;
    }

    // Write to temporary file first
    std::string tempPath = std::string(enginePath) + ".tmp";
    std::ofstream engineFile(tempPath, std::ios::binary);
    if (!engineFile) {
        return false;
    }

    // Write version and metadata
    engineFile.write(ENGINE_CACHE_VERSION, strlen(ENGINE_CACHE_VERSION));
    
    // Write engine data
    engineFile.write(
        static_cast<const char*>(serializedEngine->data()),
        serializedEngine->size()
    );
    
    engineFile.close();

    // Atomic rename for safe file writing
    return std::filesystem::rename(tempPath, enginePath);
}