#
# Copyright (c) 2023 Ogata Laboratory, Waseda University
#
# Released under the AGPL license.
# see https://www.gnu.org/licenses/agpl-3.0.txt
#

import torch
from torchvision import transforms
from torch.utils.data import Dataset, DataLoader

class MultimodalAllDataset(Dataset):
    """
    This class is used to train models that deal with multimodal data (e.g., images, joints), such as CNNRNN/SARNN.

    Args:
        images (numpy array): Set of images in the dataset, expected to be a 5D array [data_num, seq_num, channel, height, width].
        joints (numpy array): Set of joints in the dataset, expected to be a 3D array [data_num, seq_num, joint_dim].
        forces (numpy array): Set of forces in the dataset, expected to be a 3D array [data_num, seq_num, force_dim].
        proximitys (numpy array): Set of proximitys in the dataset, expected to be a 3D array [data_num, seq_num, proximity_dim].
        stdev (float, optional): Set the standard deviation for normal distribution to generate noise.
    """

    def __init__(self, image, joint, pose, force, proximity, stdev=0.0, training=True):
        """
        The constructor of Multimodal Dataset class. Initializes the images, joints, and transformation.

        Args:
            images (numpy array): The images data, expected to be a 5D array [data_num, seq_num, channel, height, width].
            joints (numpy array): The joints data, expected to be a 3D array [data_num, seq_num, joint_dim].
            poses (numpy array): The poses data, expected to be a 3D array [data_num, seq_num, pose_dim].
            forces (numpy array): Set of forces in the dataset, expected to be a 3D array [data_num, seq_num, force_dim].
            proximitys (numpy array): Set of proximitys in the dataset, expected to be a 3D array [data_num, seq_num, proximity_dim].
            stdev (float, optional): The standard deviation for the normal distribution to generate noise. Defaults to 0.02.
        """
        self.stdev = stdev
        self.training = training
        self.image = torch.from_numpy(image).float()
        self.joint = torch.from_numpy(joint).float()
        self.pose = torch.from_numpy(pose).float()

        self.force = torch.from_numpy(force).float()
        self.proximity = torch.from_numpy(proximity).float()
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
        return len(self.image)

    def __getitem__(self, idx):
        """
        Extraction and preprocessing of images and joints at the specified indexes.

        Args:
            idx (int): The index of the element.

        Returns:
            dataset (list): A list containing lists of transformed and noise added image and joint (x_img, x_joint) and the original image and joint (y_img, y_joint).
        """

        if self.training:
            y_img = self.transform_affine(self.image[idx])
            x_img = self.transform_noise(y_img) + torch.normal(mean=0, std=self.stdev, size=y_img.shape)

            def add_noise(data):
                y_data = data[idx]
                x_data = data[idx] + torch.normal(mean=0, std=self.stdev, size=y_data.shape)
                return x_data, y_data
            
            x_joint, y_joint = add_noise(self.joint)
            x_pose, y_pose = add_noise(self.pose)

            x_force, y_force = add_noise(self.force)
            x_proximity, y_proximity = add_noise(self.proximity)
        
        else:
            def x_y_maker(data):
                x = data[idx]
                y = data[idx]
                return x, y
            
            x_img, y_img = x_y_maker(self.image)
            x_joint, y_joint = x_y_maker(self.joint)
            x_pose, y_pose = x_y_maker(self.pose)

            x_force, y_force = x_y_maker(self.force)
            x_proximity, y_proximity = x_y_maker(self.proximity)

        return [[x_img, x_joint, x_pose, x_force, x_proximity],
                [y_img, y_joint, y_pose, y_force, y_proximity]]