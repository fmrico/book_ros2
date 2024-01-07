// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use std::env;
use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "logger_node")?;
  
    // ToImpl: Clocks, Time, SimTime, Rate  
    let mut counter = 0;
    while context.ok() {
      // ToImpl: Loggers
      println!("Hello {}", counter);
      counter = counter + 1;
      std::thread::sleep(std::time::Duration::from_millis(500));
  }
  Ok(())
}
