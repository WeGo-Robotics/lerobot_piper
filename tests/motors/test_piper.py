#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 WeGo-Robotics Inc. EDU team. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#TODO Implement Mock I/F

import pytest

from piper_sdk import *
from wego_piper.port_handler import *

@pytest.fixture
def port() -> PortHandler:
    _port = PortHandler()
    return _port

@pytest.fixture
def piper(port) -> C_PiperInterface_V2:
    _piper = C_PiperInterface_V2()
    port.setupPort(_piper)
    return _piper
    
def test_canbus(port, piper):
    port.openPort()
    assert port.is_open == True

def test_piper_read_status(port, piper):
    status = piper.GetArmStatus()
    assert status.arm_status.ctrl_mode == 0
    assert status.arm_status.arm_status == 0