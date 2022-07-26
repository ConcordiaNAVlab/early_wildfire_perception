#!/usr/bin/env python3
# -*- coding: utf-8 -*- #

# ------------------------------------------------------------------------------
#
#   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
#
#   @Filename: find_H20T_param.py
#
#   @Author: Shun Li
#
#   @Date: 2022-05-27
#
#   @Email: 2015097272@qq.com
#
#   @Description: 
#
# ------------------------------------------------------------------------------

import scipy.io as scio

data_file = "./matlab.mat"

param = scio.loadmat(data_file)
