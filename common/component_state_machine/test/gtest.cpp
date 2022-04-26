//
//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "component_state_machine/machine.hpp"

#include <gtest/gtest.h>

enum class MyStates : uint16_t { STATE1 };

enum class MyEvents : uint16_t { EVENT1 };

TEST(tests, case1)
{
  using component_state_machine::Machine;
  Machine<MyStates, MyEvents> machine;

  auto state = machine.GetState();
  EXPECT_EQ(static_cast<std::underlying_type_t<MyStates>>(state), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
