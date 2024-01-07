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


use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use std::env;
use anyhow::{Error, Result};
use std::thread::JoinHandle;

struct LoggerNode {
    node: Arc<rclrs::Node>,
    counter: Arc<Mutex<i32>>,
    timer_handle: Option<JoinHandle<()>>,
}

impl LoggerNode {
    fn new(context: &rclrs::Context, name: &str) -> Result<Arc<Mutex<Self>>, rclrs::RclrsError> {
        let node = rclrs::create_node(&context, name)?;

        let counter = Arc::new(Mutex::new(0));
        let counter_clone = Arc::clone(&counter);

        let logger_node = Arc::new(Mutex::new(LoggerNode {
          node: node.clone(),
          counter: counter,
          timer_handle: None,
        }));

        let logger_node_aux = Arc::clone(&logger_node);

        let handle = thread::spawn(move || loop {
            thread::sleep(Duration::from_millis(500));
            let logger_node = logger_node_aux.lock().unwrap();
            logger_node.periodic_call();
        });

        {
          let mut logger_node = logger_node.lock().unwrap();
          logger_node.timer_handle = Some(handle);
        }

        Ok(logger_node)
    }

    fn periodic_call(&self) {
        let mut num = self.counter.lock().unwrap();
        *num += 1;
        println!("Counter is {}", *num);
    }
}

fn main() -> Result<(), Error>{
    let context = rclrs::Context::new(env::args())?;
    let logger_node = LoggerNode::new(&context, "logger_node")?;
    
    let executor = rclrs::SingleThreadedExecutor::new();
    executor.add_node(&logger_node.lock().unwrap().node)?;

    executor.spin().map_err(|err| err.into())
}
