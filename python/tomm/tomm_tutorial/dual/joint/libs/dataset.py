#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import torch
from torchvision import transforms
from torch.utils.data import Dataset, DataLoader


class MultimodalDataset(Dataset):
    """
    This class is used to train models that deal with multimodal data (e.g., images, joints), such as CNNRNN/SARNN.

    Args:
        images (numpy array): Set of images in the dataset, expected to be a 5D array [data_num, seq_num, channel, height, width].
        joints (numpy array): Set of joints in the dataset, expected to be a 3D array [data_num, seq_num, joint_dim].
        stdev (float, optional): Set the standard deviation for normal distribution to generate noise.
    """

    def __init__(self, left_image, right_image, left_joint, right_joint, stdev=0.0, training=True):
        """
        The constructor of Multimodal Dataset class. Initializes the images, joints, and transformation.

        Args:
            images (numpy array): The images data, expected to be a 5D array [data_num, seq_num, channel, height, width].
            joints (numpy array): The joints data, expected to be a 3D array [data_num, seq_num, joint_dim].
            stdev (float, optional): The standard deviation for the normal distribution to generate noise. Defaults to 0.02.
        """
        self.stdev = stdev
        self.training = training
        self.left_image = torch.from_numpy(left_image).float()
        self.right_image = torch.from_numpy(right_image).float()
        self.left_joint = torch.from_numpy(left_joint).float()
        self.right_joint = torch.from_numpy(right_joint).float()
        self.transform = transforms.ColorJitter(contrast=0.5, brightness=0.5, saturation=0.1)

        self.transform_affine = transforms.Compose(
            [
                transforms.RandomAutocontrast(),
            ]
        )
        self.transform_noise = transforms.Compose(
            [
                transforms.ColorJitter(contrast=0.5, brightness=0.5),
            ]
        )

    def __len__(self):
        """
        Returns the number of the data.
        """
        return len(self.left_image)

    def __getitem__(self, idx):
        """
        Extraction and preprocessing of images and joints at the specified indexes.

        Args:
            idx (int): The index of the element.

        Returns:
            dataset (list): A list containing lists of transformed and noise added image and joint (x_img, x_joint) and the original image and joint (y_img, y_joint).
        """

        if self.training:
            y_left_img = self.transform_affine(self.left_image[idx])
            y_right_img = self.transform_affine(self.right_image[idx])
            x_left_img = self.transform_noise(y_left_img) + torch.normal(
                mean=0, std=self.stdev, size=y_left_img.shape
            )
            x_right_img = self.transform_noise(y_right_img) + torch.normal(
                mean=0, std=self.stdev, size=y_right_img.shape
            )

            y_left_joint= self.left_joint[idx]
            y_right_joint= self.right_joint[idx]
            x_left_joint = self.left_joint[idx] + torch.normal(mean=0, std=self.stdev, size=y_left_joint.shape)
            x_right_joint = self.right_joint[idx] + torch.normal(mean=0, std=self.stdev, size=y_right_joint.shape)
        
        else:
            x_left_img = self.left_image[idx]
            x_right_img = self.right_image[idx]
            x_left_joint = self.left_joint[idx]
            x_right_joint = self.right_joint[idx]
            y_left_img = self.left_image[idx]
            y_right_img = self.right_image[idx]
            y_left_joint = self.left_joint[idx]
            y_right_joint = self.right_joint[idx]

        return [[x_left_img, x_right_img, x_left_joint, x_right_joint], [y_left_img, y_right_img, y_left_joint, y_right_joint]]
    

class MultiEpochsDataLoader(DataLoader):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._DataLoader__initialized = False
        self.batch_sampler = _RepeatSampler(self.batch_sampler)
        self._DataLoader__initialized = True
        self.iterator = super().__iter__()

    def __len__(self):
        return len(self.batch_sampler.sampler)

    def __iter__(self):
        for _ in range(len(self)):
            yield next(self.iterator)

class _RepeatSampler(object):
    """Sampler that repeats forever.

    Args:
        sampler (Sampler)
    """

    def __init__(self, sampler):
        self.sampler = sampler

    def __iter__(self):
        while True:
            yield from iter(self.sampler)