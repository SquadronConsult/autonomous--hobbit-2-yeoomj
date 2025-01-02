# NVIDIA TAO Toolkit Data Loading Module v4.0.0
# Optimized for agricultural image datasets with GPU acceleration
# Dependencies:
# torch==1.13.0
# numpy==1.23.5
# albumentations==1.3.0
# opencv-python==4.7.0

import torch
from torch.utils.data import Dataset, DataLoader as TorchDataLoader
import numpy as np
import albumentations as A
import cv2
from typing import Tuple, Dict, List, Optional
import os
from collections import OrderedDict
import logging
from concurrent.futures import ThreadPoolExecutor

from ..models.pestDetection.config import training_config, augmentation
from ..models.cropAnalysis.config import augmentation_config

class TAODataset(Dataset):
    """Enhanced PyTorch Dataset implementation for TAO Toolkit with memory optimization."""
    
    def __init__(
        self,
        data_dir: str,
        model_type: str,
        split: str = "train",
        use_augmentation: bool = True,
        cache_size: int = 1000,
        pin_memory: bool = True
    ) -> None:
        """Initialize dataset with advanced caching and memory management.
        
        Args:
            data_dir: Root directory containing image data
            model_type: Type of model ('pest_detection' or 'crop_analysis')
            split: Dataset split ('train' or 'val')
            use_augmentation: Enable data augmentation
            cache_size: Number of images to cache in memory
            pin_memory: Pin memory for faster GPU transfer
        """
        super().__init__()
        self.data_dir = data_dir
        self.model_type = model_type
        self.split = split
        self.use_augmentation = use_augmentation
        self.pin_memory = pin_memory
        
        # Initialize LRU cache
        self.image_cache = OrderedDict()
        self.cache_size = cache_size
        self.cache_hits = 0
        self.cache_misses = 0
        
        # Load configuration based on model type
        self.config = self._load_config()
        
        # Setup data paths and validation
        self.image_paths, self.annotations = self._load_dataset()
        
        # Initialize augmentation pipeline
        self.transform_pipeline = self._create_transform_pipeline()
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def _load_config(self) -> Dict:
        """Load and validate model configuration."""
        if self.model_type == "pest_detection":
            config = {
                "augmentation": augmentation,
                "training": training_config
            }
        elif self.model_type == "crop_analysis":
            config = {
                "augmentation": augmentation_config
            }
        else:
            raise ValueError(f"Unsupported model type: {self.model_type}")
        return config

    def _create_transform_pipeline(self) -> A.Compose:
        """Create dynamic augmentation pipeline based on configuration."""
        transforms = []
        
        if self.use_augmentation and self.split == "train":
            aug_config = self.config["augmentation"]
            
            # Add augmentation transforms based on config
            if self.model_type == "pest_detection":
                transforms.extend([
                    A.HorizontalFlip(p=0.5),
                    A.Rotate(limit=aug_config.get("rotation", 15), p=0.3),
                    A.ColorJitter(
                        brightness=aug_config["color_jitter"]["brightness"],
                        contrast=aug_config["color_jitter"]["contrast"],
                        saturation=aug_config["color_jitter"]["saturation"],
                        p=0.3
                    )
                ])
            else:  # crop_analysis
                for transform in aug_config["transforms"]:
                    if transform["name"] == "RandomHorizontalFlip":
                        transforms.append(A.HorizontalFlip(p=transform["probability"]))
                    elif transform["name"] == "RandomRotate":
                        transforms.append(A.Rotate(
                            limit=transform["parameters"]["angle_range"],
                            p=transform["probability"]
                        ))
                    elif transform["name"] == "RandomBrightnessContrast":
                        transforms.append(A.RandomBrightnessContrast(
                            brightness_limit=transform["parameters"]["brightness_limit"],
                            contrast_limit=transform["parameters"]["contrast_limit"],
                            p=transform["probability"]
                        ))
                    elif transform["name"] == "RandomCrop":
                        transforms.append(A.RandomCrop(
                            height=transform["parameters"]["min_height"],
                            width=transform["parameters"]["min_width"],
                            p=transform["probability"]
                        ))

        # Add normalization as final transform
        transforms.append(A.Normalize(
            mean=self.config["augmentation"]["parameters"]["normalize"]["mean"],
            std=self.config["augmentation"]["parameters"]["normalize"]["std"]
        ))
        
        return A.Compose(transforms, bbox_params=A.BboxParams(
            format='pascal_voc',
            label_fields=['class_labels']
        ))

    def _load_dataset(self) -> Tuple[List[str], List[Dict]]:
        """Load and validate dataset paths and annotations."""
        image_dir = os.path.join(self.data_dir, self.split, "images")
        label_dir = os.path.join(self.data_dir, self.split, "labels")
        
        if not os.path.exists(image_dir) or not os.path.exists(label_dir):
            raise FileNotFoundError(f"Dataset directories not found: {image_dir}, {label_dir}")
        
        image_paths = sorted([
            os.path.join(image_dir, f) for f in os.listdir(image_dir)
            if f.endswith(('.jpg', '.jpeg', '.png'))
        ])
        
        annotations = []
        for img_path in image_paths:
            base_name = os.path.splitext(os.path.basename(img_path))[0]
            label_path = os.path.join(label_dir, f"{base_name}.txt")
            
            if os.path.exists(label_path):
                with open(label_path, 'r') as f:
                    annotations.append(self._parse_annotation(f.readlines()))
            else:
                raise FileNotFoundError(f"Annotation file not found: {label_path}")
        
        return image_paths, annotations

    def _parse_annotation(self, lines: List[str]) -> Dict:
        """Parse annotation file into structured format."""
        boxes = []
        labels = []
        
        for line in lines:
            data = line.strip().split()
            labels.append(int(data[0]))
            # Convert from YOLO to Pascal VOC format
            x_center, y_center, w, h = map(float, data[1:5])
            x_min = x_center - w/2
            y_min = y_center - h/2
            x_max = x_center + w/2
            y_max = y_center + h/2
            boxes.append([x_min, y_min, x_max, y_max])
        
        return {
            'boxes': np.array(boxes, dtype=np.float32),
            'labels': np.array(labels, dtype=np.int64)
        }

    def __len__(self) -> int:
        """Get dataset size."""
        return len(self.image_paths)

    def __getitem__(self, index: int) -> Tuple[torch.Tensor, Dict]:
        """Get a single data sample with caching and error handling."""
        try:
            # Check cache first
            if index in self.image_cache:
                self.cache_hits += 1
                image = self.image_cache[index]
            else:
                self.cache_misses += 1
                # Load and validate image
                image_path = self.image_paths[index]
                image = cv2.imread(image_path)
                if image is None:
                    raise ValueError(f"Failed to load image: {image_path}")
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                
                # Update cache
                if len(self.image_cache) >= self.cache_size:
                    self.image_cache.popitem(last=False)
                self.image_cache[index] = image

            # Get annotations
            annotation = self.annotations[index]
            
            # Apply augmentation pipeline
            transformed = self.transform_pipeline(
                image=image,
                bboxes=annotation['boxes'],
                class_labels=annotation['labels']
            )
            
            # Prepare final tensors
            image_tensor = torch.from_numpy(
                transformed['image'].transpose(2, 0, 1)
            ).float()
            
            target = {
                'boxes': torch.from_numpy(np.array(transformed['bboxes'])),
                'labels': torch.from_numpy(np.array(transformed['class_labels']))
            }
            
            # Pin memory if enabled
            if self.pin_memory:
                image_tensor = image_tensor.pin_memory()
                target = {k: v.pin_memory() for k, v in target.items()}
            
            return image_tensor, target
            
        except Exception as e:
            self.logger.error(f"Error processing index {index}: {str(e)}")
            raise

    def cleanup_cache(self) -> None:
        """Clean up cached resources."""
        self.image_cache.clear()
        torch.cuda.empty_cache()

class DataLoader(TorchDataLoader):
    """Optimized data loader for efficient batch processing with GPU acceleration."""
    
    def __init__(
        self,
        dataset: TAODataset,
        batch_size: int = 32,
        shuffle: bool = True,
        num_workers: int = 4,
        prefetch_factor: int = 2,
        persistent_workers: bool = True
    ) -> None:
        """Initialize optimized data loader with advanced features."""
        super().__init__(
            dataset=dataset,
            batch_size=batch_size,
            shuffle=shuffle,
            num_workers=num_workers,
            prefetch_factor=prefetch_factor,
            persistent_workers=persistent_workers,
            pin_memory=dataset.pin_memory,
            collate_fn=self._collate_fn
        )

    @staticmethod
    def _collate_fn(batch: List[Tuple]) -> Tuple[torch.Tensor, List[Dict]]:
        """Custom collate function for handling variable-sized data."""
        images = torch.stack([item[0] for item in batch])
        targets = [item[1] for item in batch]
        return images, targets

def create_data_loaders(
    data_dir: str,
    model_type: str,
    loader_config: Dict,
    val_split: float = 0.2,
    use_cross_validation: bool = False
) -> Tuple[DataLoader, DataLoader]:
    """Create optimized train and validation data loaders."""
    # Validate inputs
    if not os.path.exists(data_dir):
        raise FileNotFoundError(f"Data directory not found: {data_dir}")
    
    # Calculate optimal worker count
    num_workers = min(
        os.cpu_count() or 1,
        loader_config.get('num_workers', 4)
    )
    
    # Create datasets
    train_dataset = TAODataset(
        data_dir=data_dir,
        model_type=model_type,
        split="train",
        use_augmentation=True,
        cache_size=loader_config.get('cache_size', 1000),
        pin_memory=True
    )
    
    val_dataset = TAODataset(
        data_dir=data_dir,
        model_type=model_type,
        split="val",
        use_augmentation=False,
        cache_size=loader_config.get('cache_size', 1000),
        pin_memory=True
    )
    
    # Create data loaders
    train_loader = DataLoader(
        dataset=train_dataset,
        batch_size=loader_config.get('batch_size', 32),
        shuffle=True,
        num_workers=num_workers,
        prefetch_factor=loader_config.get('prefetch_factor', 2),
        persistent_workers=True
    )
    
    val_loader = DataLoader(
        dataset=val_dataset,
        batch_size=loader_config.get('batch_size', 32),
        shuffle=False,
        num_workers=num_workers,
        prefetch_factor=loader_config.get('prefetch_factor', 2),
        persistent_workers=True
    )
    
    return train_loader, val_loader