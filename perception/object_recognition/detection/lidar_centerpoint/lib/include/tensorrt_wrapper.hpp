// Copyright 2021 Tier IV, Inc.
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

#ifndef TENSORRT_WRAPPER_HPP_
#define TENSORRT_WRAPPER_HPP_

#include <NvInfer.h>

#include <iostream>
#include <memory>
#include <string>

namespace centerpoint
{
struct Deleter
{
  template <typename T>
  void operator()(T * obj) const
  {
    if (obj) {
      obj->destroy();
    }
  }
};

template <typename T>
using unique_ptr = std::unique_ptr<T, Deleter>;

class Logger : public nvinfer1::ILogger
{
public:
  explicit Logger(bool verbose) : verbose_(verbose) {}

  void log(Severity severity, const char * msg) noexcept override
  {
    if (verbose_ || ((severity != Severity::kINFO) && (severity != Severity::kVERBOSE))) {
      std::cout << msg << std::endl;
    }
  }

private:
  bool verbose_{false};
};

class TensorRTWrapper
{
public:
  explicit TensorRTWrapper(bool verbose);

  bool init(
    const std::string & onnx_path, const std::string & engine_path, const std::string & precision);

  unique_ptr<nvinfer1::IExecutionContext> context_ = nullptr;

protected:
  virtual bool setProfile(
    nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
    nvinfer1::IBuilderConfig & config) = 0;

private:
  bool parseONNX(
    const std::string & onnx_path, const std::string & engine_path, const std::string & precision,
    size_t workspace_size = (1ULL << 30));

  bool saveEngine(const std::string & engine_path);

  bool loadEngine(const std::string & engine_path);

  bool createContext();

  Logger logger_;
  unique_ptr<nvinfer1::IRuntime> runtime_ = nullptr;
  unique_ptr<nvinfer1::ICudaEngine> engine_ = nullptr;
};

}  // namespace centerpoint

#endif  // TENSORRT_WRAPPER_HPP_
