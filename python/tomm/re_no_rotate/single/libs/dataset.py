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

    def __init__(self, images, poses, forces, prox, stdev=None):
        """
        The constructor of Multimodal Dataset class. Initializes the images, joints, and transformation.

        Args:
            images (numpy array): The images data, expected to be a 5D array [data_num, seq_num, channel, height, width].
            joints (numpy array): The joints data, expected to be a 3D array [data_num, seq_num, joint_dim].
            stdev (float, optional): The standard deviation for the normal distribution to generate noise. Defaults to 0.02.
        """
        self.stdev = stdev
        self.images = torch.from_numpy(images).float()
        self.poses = torch.from_numpy(poses).float()
        self.forces = torch.from_numpy(forces).float()
        self.proxs = torch.from_numpy(prox).float()

        self.transform = transforms.ColorJitter(
            contrast=[0.6, 1.4], brightness=0.4, saturation=[0.6, 1.4], hue=0.04
        )

    def __len__(self):
        """
        Returns the number of the data.
        """
        return len(self.images)

    def __getitem__(self, idx):
        """
        Extraction and preprocessing of images and joints at the specified indexes.

        Args:
            idx (int): The index of the element.

        Returns:
            dataset (list): A list containing lists of transformed and noise added image and joint (x_img, x_joint) and the original image and joint (y_img, y_joint).
        """
        y_img = self.images[idx]
        y_pose = self.poses[idx]
        y_force = self.forces[idx]
        y_prox = self.proxs[idx]

        if self.stdev is not None:
            x_img = self.transform(y_img)
            x_img = x_img + torch.normal(mean=0, std=0.02, size=x_img.shape)
            x_pose = y_pose + torch.normal(mean=0, std=self.stdev, size=y_pose.shape)
            x_force = y_force + torch.normal(mean=0, std=self.stdev, size=y_force.shape)
            x_prox = y_prox + torch.normal(mean=0, std=self.stdev, size=y_prox.shape)

        else:
            x_img = y_img
            x_pose = y_pose
            x_force = y_force
            x_prox = y_prox

        return [[x_img, x_pose, x_force, x_prox], [y_img, y_pose, y_force, y_prox]]


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