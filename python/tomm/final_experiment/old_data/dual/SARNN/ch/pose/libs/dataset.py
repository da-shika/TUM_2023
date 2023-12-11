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

    def __init__(self, left_images, right_images, left_poses, right_poses, stdev=None):
        """
        The constructor of Multimodal Dataset class. Initializes the images, joints, and transformation.

        Args:
            images (numpy array): The images data, expected to be a 5D array [data_num, seq_num, channel, height, width].
            joints (numpy array): The joints data, expected to be a 3D array [data_num, seq_num, joint_dim].
            stdev (float, optional): The standard deviation for the normal distribution to generate noise. Defaults to 0.02.
        """
        self.stdev = stdev
        self.left_images = torch.from_numpy(left_images).float()
        self.right_images = torch.from_numpy(right_images).float()
        self.left_poses = torch.from_numpy(left_poses).float()
        self.right_poses = torch.from_numpy(right_poses).float()

        self.transform = transforms.ColorJitter(
            contrast=[0.6, 1.4], brightness=0.4, saturation=[0.6, 1.4], hue=0.04
        )

    def __len__(self):
        """
        Returns the number of the data.
        """
        return len(self.left_images)

    def __getitem__(self, idx):
        """
        Extraction and preprocessing of images and joints at the specified indexes.

        Args:
            idx (int): The index of the element.

        Returns:
            dataset (list): A list containing lists of transformed and noise added image and joint (x_img, x_joint) and the original image and joint (y_img, y_joint).
        """
        y_left_img = self.left_images[idx]
        y_left_pose = self.left_poses[idx]
        y_right_img = self.right_images[idx]
        y_right_pose = self.right_poses[idx]

        if self.stdev is not None:
            x_left_img = self.transform(y_left_img)
            x_left_img = x_left_img + torch.normal(mean=0, std=0.02, size=x_left_img.shape)
            x_left_pose = y_left_pose + torch.normal(mean=0, std=self.stdev, size=y_left_pose.shape)

            x_right_img = self.transform(y_right_img)
            x_right_img = x_right_img + torch.normal(mean=0, std=0.02, size=x_right_img.shape)
            x_right_pose = y_right_pose + torch.normal(mean=0, std=self.stdev, size=y_right_pose.shape)

        else:
            x_left_img = y_left_img
            x_left_pose = y_left_pose
            x_right_img = y_right_img
            x_right_pose = y_right_pose

        return [[x_left_img, x_right_img, x_left_pose, x_right_pose],
                [y_left_img, y_right_img, y_left_pose, y_right_pose]]


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