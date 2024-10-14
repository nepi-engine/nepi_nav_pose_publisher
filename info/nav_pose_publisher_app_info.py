#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

APP_NAME = 'Nav_Pose_Publisher' # Use in display menus
FILE_TYPE = 'APP'
APP_DICT = dict(
    description = 'Application for publishing the current NEPI nav pose solution in different standard formats',
    pkg_name = 'nepi_app_navpose_publisher',
    group_name = 'NAVPOSE',
    config_file = 'app_nav_pose_publisher.yaml',
    app_file = 'nav_pose_publisher_node.py',
    node_name = 'nav_pose_publisher_app'
)
RUI_DICT = dict(
    rui_menu_name = 'None', # RUI menu name or "None" if no rui support
    rui_files = ['None'],
    rui_main_file ='None',
    rui_main_class = 'None'
)



