#!/usr/bin/env python3
# encoding: utf-8

import cv2
import numpy as np
import torch


def normalize(img, mean, std):
    # pytorch pretrained model need the input range: 0-1
    img = img.astype(np.float32) / 255.0
    img = img - mean
    img = img / std

    return img


def process_image(img, config, crop_size=None):
    p_img = img

    if img.shape[2] < 3:
        im_b = p_img
        im_g = p_img
        im_r = p_img
        p_img = np.concatenate((im_b, im_g, im_r), axis=2)

    p_img = normalize(p_img, config.image_mean, config.image_std)

    p_img = p_img.transpose(2, 0, 1)

    return p_img


def val_func_process(input_data, model, device=None):
    input_data = np.ascontiguousarray(input_data[None, :, :, :],
                                      dtype=np.float32)
    input_data = torch.tensor(input_data, device=device)

    with torch.cuda.device(input_data.get_device()):
        model.eval()
        model.to(input_data.get_device())
        with torch.no_grad():
            score = model(input_data)
            score = score[0]

            score = torch.exp(score)

    return score


def predict(config, model, img, device):
    input_size = None
    output_size = (config.image_height // config.gt_down_sampling, config.image_width // config.gt_down_sampling)

    im_size = img.shape

    if (im_size[1] >= config.image_width) or (im_size[0] >= config.image_height):
        img = cv2.resize(img, (config.image_width, config.image_height),
                         interpolation=cv2.INTER_LINEAR)
        img_new = img
    img = process_image(img, config, input_size)
    pred = val_func_process(img, model, device)

    pred = pred.permute(1, 2, 0)
    pred = pred.cpu().numpy()
    if output_size is not None:
        pred = cv2.resize(pred,
                          (output_size[1], output_size[0]),
                          interpolation=cv2.INTER_LINEAR)

    pred = pred.argmax(2)

    return pred, img_new

