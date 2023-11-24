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

    def __init__(self, left_image, right_image, left_joint, right_joint, left_pose, right_pose,\
                 left_force, right_force, left_proximity, right_proximity, stdev=0.0, training=True):
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
        self.left_image = torch.from_numpy(left_image).float()
        self.right_image = torch.from_numpy(right_image).float()
        self.left_joint = torch.from_numpy(left_joint).float()
        self.right_joint = torch.from_numpy(right_joint).float()
        self.left_pose = torch.from_numpy(left_pose).float()
        self.right_pose = torch.from_numpy(right_pose).float()

        self.left_force = torch.from_numpy(left_force).float()
        self.right_force = torch.from_numpy(right_force).float()
        self.left_proximity = torch.from_numpy(left_proximity).float()
        self.right_proximity = torch.from_numpy(right_proximity).float()
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

            def add_noise(data):
                y_data = data[idx]
                x_data = data[idx] + torch.normal(mean=0, std=self.stdev, size=y_data.shape)
                return x_data, y_data
            
            x_left_joint, y_left_joint = add_noise(self.left_joint)
            x_right_joint, y_right_joint = add_noise(self.right_joint)
            x_left_pose, y_left_pose = add_noise(self.left_pose)
            x_right_pose, y_right_pose = add_noise(self.right_pose)

            x_left_force, y_left_force = add_noise(self.left_force)
            x_right_force, y_right_force = add_noise(self.right_force)
            x_left_proximity, y_left_proximity = add_noise(self.left_proximity)
            x_right_proximity, y_right_proximity = add_noise(self.right_proximity)
        
        else:
            def x_y_maker(data):
                x = data[idx]
                y = data[idx]
                return x, y
            
            x_left_img, y_left_img = x_y_maker(self.left_image)
            x_right_img, y_right_img = x_y_maker(self.right_image)
            x_left_joint, y_left_joint = x_y_maker(self.left_joint)
            x_right_joint, y_right_joint = x_y_maker(self.right_joint)
            x_left_pose, y_left_pose = x_y_maker(self.left_pose)
            x_right_pose, y_right_pose = x_y_maker(self.right_pose)

            x_left_force, y_left_force = x_y_maker(self.left_force)
            x_right_force, y_right_force = x_y_maker(self.right_force)
            x_left_proximity, y_left_proximity = x_y_maker(self.left_proximity)
            x_right_proximity, y_right_proximity = x_y_maker(self.right_proximity)

        return [[x_left_img, x_right_img, x_left_joint, x_right_joint, x_left_pose, x_right_pose, \
                 x_left_force, x_right_force, x_left_proximity, x_right_proximity],
                [y_left_img, y_right_img, y_left_joint, y_right_joint, y_left_pose, y_right_pose, \
                 y_left_force, y_right_force, y_left_proximity, y_right_proximity]]