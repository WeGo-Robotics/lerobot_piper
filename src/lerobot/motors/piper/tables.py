#!/usr/bin/env python

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



MODEL_BAUDRATE_TABLE = {
    1_000_000: 3,
    2_000_000: 4,
    3_000_000: 5,
    4_000_000: 6,
}

# {data_name: size_byte}
MODEL_ENCODINGS_TABLE = {
}

MODEL_ENCODING_TABLE = {
    "HTDW-5047": MODEL_ENCODINGS_TABLE,
}

# {model: model_resolution}
MODEL_RESOLUTION_TABLE = {
    "HTDW-5047": 4096,
}

# {model: model_number}
MODEL_NUMBER_TABLE = {
    "HTDW-5047": 1190,
}

# {model: available_operating_modes}
MODEL_OPERATING_MODES = {
    "HTDW-5047": [0, 1, 3, 4, 5, 16],
}

MODEL_CONTROL_TABLE = {
    "HTDW-5047": 1190,
}

MODEL_BAUDRATE_TABLE = {
    "HTDW-5047": 1190,
}

AVAILABLE_BAUDRATES = [
    9_600,
    19_200,
    38_400,
    57_600,
    115_200,
    230_400,
    460_800,
    500_000,
    576_000,
    921_600,
    1_000_000,
    1_152_000,
    2_000_000,
    2_500_000,
    3_000_000,
    3_500_000,
    4_000_000,
]
