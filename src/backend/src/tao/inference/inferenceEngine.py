"""
NVIDIA TAO Toolkit Inference Engine for Agricultural Pest Detection
Optimized for Jetson Orin deployment with TensorRT acceleration

Version: 1.0.0
Requirements:
- torch==1.13.0
- tensorrt==8.5.0
- numpy==1.23.5
- pycuda==2022.1
"""

import os
import time
import logging
from typing import Dict, List, Optional, Tuple, Union

import numpy as np
import torch
import tensorrt as trt
import pycuda.driver as cuda
from pest_detection_config import inference_config, export_config

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class TAOInferenceEngine:
    """High-performance inference engine for TAO Toolkit optimized models with comprehensive 
    resource management and monitoring for agricultural pest detection.
    """
    
    def __init__(self, 
                 model_path: str, 
                 inference_config: Dict, 
                 enable_profiling: bool = False) -> None:
        """Initialize the TAO inference engine with optimized settings for Jetson Orin.

        Args:
            model_path: Path to the TensorRT engine file
            inference_config: Configuration for inference parameters
            enable_profiling: Enable detailed performance profiling
        """
        self.model_path = model_path
        self.config = inference_config
        self._validate_inputs()
        
        # Initialize CUDA
        cuda.init()
        self.device = cuda.Device(0)  # Use primary GPU
        self.cuda_ctx = self.device.make_context()
        self.stream = cuda.Stream()
        
        # Initialize TensorRT
        logger.info("Initializing TensorRT engine...")
        trt.init_libnvinfer_plugins(None, "")
        self.runtime = trt.Runtime(trt.Logger(trt.Logger.INFO))
        
        # Load engine
        with open(self.model_path, 'rb') as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())
        
        if not self.engine:
            raise RuntimeError("Failed to load TensorRT engine")
            
        self.context = self.engine.create_execution_context()
        
        # Allocate memory
        self._allocate_buffers()
        
        # Initialize performance monitoring
        self.performance_metrics = {
            'inference_time': [],
            'preprocessing_time': [],
            'postprocessing_time': [],
            'total_time': [],
            'batch_size': [],
            'gpu_memory_used': 0
        }
        
        # Initialize profiler if enabled
        self.profiler = None
        if enable_profiling:
            self.profiler = trt.Profiler()
            self.context.profiler = self.profiler
            
        # Perform warmup
        self._warmup()

    def _validate_inputs(self) -> None:
        """Validate input parameters and configuration."""
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model file not found: {self.model_path}")
        
        required_config_keys = ['confidence_threshold', 'nms_threshold', 
                              'preprocessing', 'postprocessing']
        for key in required_config_keys:
            if key not in self.config:
                raise ValueError(f"Missing required config key: {key}")

    def _allocate_buffers(self) -> None:
        """Allocate GPU memory for input and output tensors."""
        self.input_tensors = []
        self.output_tensors = []
        
        # Allocate memory for inputs
        for binding in range(self.engine.num_bindings):
            shape = self.engine.get_binding_shape(binding)
            size = trt.volume(shape) * self.engine.max_batch_size
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            
            # Allocate host and device memory
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            
            if self.engine.binding_is_input(binding):
                self.input_tensors.append({
                    'host': host_mem,
                    'device': device_mem,
                    'shape': shape,
                    'name': self.engine.get_binding_name(binding)
                })
            else:
                self.output_tensors.append({
                    'host': host_mem,
                    'device': device_mem,
                    'shape': shape,
                    'name': self.engine.get_binding_name(binding)
                })

    def _warmup(self, num_warmup: int = 5) -> None:
        """Perform warmup iterations to initialize GPU kernels.
        
        Args:
            num_warmup: Number of warmup iterations
        """
        logger.info("Performing warmup iterations...")
        dummy_input = np.random.randn(*self.input_tensors[0]['shape']).astype(np.float32)
        
        for _ in range(num_warmup):
            self.infer(dummy_input)
            
        # Clear warmup metrics
        self.performance_metrics['inference_time'].clear()
        cuda.Context.synchronize()

    def preprocess(self, input_data: np.ndarray, batch_size: int) -> np.ndarray:
        """Preprocess input data for inference.
        
        Args:
            input_data: Input image data
            batch_size: Batch size for processing
            
        Returns:
            Preprocessed input tensor
        """
        start_time = time.perf_counter()
        
        # Validate input
        if input_data.ndim != 4:
            raise ValueError("Input must be 4D: [batch_size, channels, height, width]")
            
        if batch_size > self.engine.max_batch_size:
            raise ValueError(f"Batch size exceeds maximum: {self.engine.max_batch_size}")
        
        # Apply preprocessing
        mean = np.array(self.config['preprocessing']['mean'], dtype=np.float32)
        std = np.array(self.config['preprocessing']['std'], dtype=np.float32)
        
        processed = (input_data - mean.reshape(1, 3, 1, 1)) / std.reshape(1, 3, 1, 1)
        
        self.performance_metrics['preprocessing_time'].append(
            time.perf_counter() - start_time)
        
        return processed.astype(np.float32)

    def infer(self, input_tensor: np.ndarray) -> Dict:
        """Execute inference with performance monitoring.
        
        Args:
            input_tensor: Preprocessed input tensor
            
        Returns:
            Dictionary containing inference results and metrics
        """
        start_time = time.perf_counter()
        
        try:
            # Copy input to GPU
            np.copyto(self.input_tensors[0]['host'], input_tensor.ravel())
            cuda.memcpy_htod_async(self.input_tensors[0]['device'], 
                                 self.input_tensors[0]['host'], 
                                 self.stream)
            
            # Run inference
            bindings = [t['device'] for t in self.input_tensors + self.output_tensors]
            self.context.execute_async_v2(bindings, self.stream.handle)
            
            # Copy outputs from GPU
            outputs = {}
            for tensor in self.output_tensors:
                cuda.memcpy_dtoh_async(tensor['host'], tensor['device'], self.stream)
                
            self.stream.synchronize()
            
            # Format outputs
            for tensor in self.output_tensors:
                outputs[tensor['name']] = tensor['host'].reshape(tensor['shape'])
            
            inference_time = time.perf_counter() - start_time
            self.performance_metrics['inference_time'].append(inference_time)
            
            # Update GPU memory usage
            self.performance_metrics['gpu_memory_used'] = \
                cuda.mem_get_info()[1] - cuda.mem_get_info()[0]
            
            return outputs
            
        except cuda.Error as e:
            logger.error(f"CUDA error during inference: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Error during inference: {str(e)}")
            raise

    def postprocess(self, inference_outputs: Dict) -> Dict:
        """Post-process inference outputs.
        
        Args:
            inference_outputs: Raw inference outputs
            
        Returns:
            Processed detection results
        """
        start_time = time.perf_counter()
        
        try:
            boxes = inference_outputs['boxes']
            scores = inference_outputs['scores']
            classes = inference_outputs['classes']
            
            # Apply confidence threshold
            conf_mask = scores > self.config['postprocessing']['score_threshold']
            boxes = boxes[conf_mask]
            scores = scores[conf_mask]
            classes = classes[conf_mask]
            
            # Apply NMS
            keep_indices = self._apply_nms(
                boxes, scores, self.config['postprocessing']['iou_threshold'])
            
            results = {
                'boxes': boxes[keep_indices],
                'scores': scores[keep_indices],
                'classes': classes[keep_indices]
            }
            
            self.performance_metrics['postprocessing_time'].append(
                time.perf_counter() - start_time)
            
            return results
            
        except Exception as e:
            logger.error(f"Error during postprocessing: {str(e)}")
            raise

    def _apply_nms(self, boxes: np.ndarray, scores: np.ndarray, 
                  iou_threshold: float) -> np.ndarray:
        """Apply Non-Maximum Suppression to detection boxes.
        
        Args:
            boxes: Detection boxes
            scores: Detection scores
            iou_threshold: IoU threshold for NMS
            
        Returns:
            Indices of kept boxes
        """
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        
        keep = []
        order = scores.argsort()[::-1]
        
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            if order.size == 1:
                break
                
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            ids = np.where(ovr <= iou_threshold)[0]
            order = order[ids + 1]
            
        return np.array(keep)

    def cleanup(self) -> None:
        """Release allocated resources."""
        try:
            # Free GPU memory
            for tensor in self.input_tensors + self.output_tensors:
                tensor['device'].free()
            
            # Release CUDA resources
            self.stream.synchronize()
            self.cuda_ctx.pop()
            
            # Clear performance metrics
            self.performance_metrics.clear()
            
            logger.info("Successfully cleaned up resources")
            
        except Exception as e:
            logger.error(f"Error during cleanup: {str(e)}")
            raise

def create_inference_engine(model_path: str, 
                          inference_config: Dict, 
                          enable_profiling: bool = False) -> TAOInferenceEngine:
    """Factory function to create and initialize inference engine instance.
    
    Args:
        model_path: Path to the TensorRT engine file
        inference_config: Configuration for inference parameters
        enable_profiling: Enable detailed performance profiling
        
    Returns:
        Configured TAOInferenceEngine instance
    """
    try:
        engine = TAOInferenceEngine(
            model_path=model_path,
            inference_config=inference_config,
            enable_profiling=enable_profiling
        )
        logger.info("Successfully created inference engine")
        return engine
        
    except Exception as e:
        logger.error(f"Failed to create inference engine: {str(e)}")
        raise