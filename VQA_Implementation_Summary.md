# VQA (Visual Question Answering) Implementation

## Overview
I have successfully implemented a VQA command for the XIAO ESP32S3 device that coordinates image capture and audio recording for Visual Question Answering scenarios.

## Key Features

### VQA Command Functionality
When the device receives a "VQA" command via BLE or serial:
1. **Starts background audio recording** (5 seconds) in a separate FreeRTOS task
2. **Simultaneously captures an image** while audio is recording
3. **Transmits the image via BLE** while audio continues recording in background
4. **Waits for audio recording to complete** (if still in progress)
5. **Transmits the recorded audio via BLE** after image is sent
6. **Sends completion signal** to indicate VQA operation is finished

## Technical Implementation

### Architecture Changes
- **Added VQA State Management**: Global `VQAState` struct to coordinate operations
- **New VQA Task**: Dedicated FreeRTOS task for orchestrating VQA operations
- **Background Audio Task**: Separate task for non-blocking audio recording
- **Extended BLE Protocol**: New message types for VQA operations

### Data Flow
```
VQA Command → VQA Task → Background Audio Task (Core 0)
              ↓
              Image Capture & Transmission
              ↓
              Wait for Audio Completion
              ↓
              Audio Transmission
              ↓
              VQA_COMPLETE Signal
```

### BLE Message Protocol
- `VQA_IMG_START:<size>` - Image transmission start with size
- `<image_data_chunks>` - Image data in chunks
- `VQA_IMG_END` - Image transmission complete
- `VQA_AUD_START:<size>` - Audio transmission start with size
- `<audio_data_chunks>` - Audio data in chunks (WAV format)
- `VQA_AUD_END` - Audio transmission complete
- `VQA_COMPLETE` - Entire VQA operation finished

### Memory Management
- **Smart cleanup**: Automatic memory deallocation for both image and audio buffers
- **Task lifecycle**: Background audio task auto-deletes after completion
- **Error handling**: Proper cleanup on failures

### Performance Optimizations
- **Parallel processing**: Audio recording happens simultaneously with image operations
- **Core distribution**: Background audio task runs on Core 0, main tasks on Core 1
- **Priority management**: Audio task gets higher priority to ensure recording quality
- **Increased stack size**: VQA task allocated 20KB stack for handling large operations

## Usage

### BLE Commands
```
MTU:512          // Negotiate larger chunk size for faster transfer
VQA              // Start VQA operation (image + audio)
```

### Serial Commands (for testing)
```
VQA              // Start VQA operation
STATUS           // Check system status including VQA queue
DEBUG            // Toggle debug logging
```

## Benefits of This Implementation

1. **True Parallelism**: Audio recording happens in background while image is being transmitted
2. **Efficient Resource Usage**: Separate cores handle different operations
3. **Robust Error Handling**: Comprehensive cleanup and error recovery
4. **Extensible Design**: Easy to modify recording duration or add new features
5. **Memory Efficient**: Proper buffer management prevents memory leaks
6. **Client-Friendly**: Clear message protocol makes client implementation straightforward

## Technical Considerations

### Timing
- **Audio Recording**: 5 seconds (configurable)
- **Image Capture**: ~100-200ms for HD resolution
- **Image Transmission**: ~2-5 seconds (depends on BLE MTU and image size)
- **Audio Transmission**: ~3-6 seconds (depends on BLE MTU)

### Memory Usage
- **Image Buffer**: ~200-500KB (HD JPEG, stored in PSRAM)
- **Audio Buffer**: ~160KB (5 seconds, 16kHz, 16-bit mono WAV)
- **VQA Task Stack**: 20KB
- **Background Audio Task**: 8KB

### Concurrency Safety
- **Atomic state flags**: Prevent race conditions between tasks
- **Proper synchronization**: VQA task waits for audio completion
- **Resource isolation**: Each task manages its own resources

This implementation provides a robust, efficient solution for VQA scenarios where both visual and audio input need to be captured and transmitted to a processing system.
