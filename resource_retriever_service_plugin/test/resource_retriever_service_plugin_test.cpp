// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include "resource_retriever_service_plugin/resource_retriever_service_plugin.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>
#include <resource_retriever/resource.hpp>
#include <resource_retriever_interfaces/srv/get_resource.hpp>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace resource_retriever_service_plugin
{
namespace
{

using ::resource_retriever_interfaces::srv::GetResource;
using ::testing::_;
using ::testing::Invoke;
using ::testing::MockFunction;
using ::testing::Return;

TEST(RosServiceResourceRetriever, GoodConstruction)
{
  auto node = rclcpp::Node::make_shared("test_message_passing");
  RosServiceResourceRetriever retriever(*node);
}

TEST(RosServiceResourceRetriever, CanHandleUri)
{
  auto node = rclcpp::Node::make_shared("test_get_shared");
  RosServiceResourceRetriever retriever(*node);

  EXPECT_FALSE(retriever.can_handle("something"));
  EXPECT_FALSE(retriever.can_handle("http://something"));
  EXPECT_FALSE(retriever.can_handle("file://something"));
  EXPECT_FALSE(retriever.can_handle("package://something"));
  EXPECT_FALSE(retriever.can_handle("service"));
  EXPECT_FALSE(retriever.can_handle("service://"));

  EXPECT_FALSE(retriever.can_handle("service://:"));
  EXPECT_FALSE(retriever.can_handle("service://:b"));
  EXPECT_FALSE(retriever.can_handle("service://a:b:c"));

  EXPECT_TRUE(retriever.can_handle("service://a:b"));
  EXPECT_TRUE(retriever.can_handle("service://a:"));
}

TEST(RosServiceResourceRetriever, BadGetSharedCall)
{
  auto node = rclcpp::Node::make_shared("test_get_shared");
  RosServiceResourceRetriever retriever(*node);

  EXPECT_EQ(nullptr, retriever.get_shared("something"));
  EXPECT_EQ(nullptr, retriever.get_shared("http://something"));
  EXPECT_EQ(nullptr, retriever.get_shared("file://something"));
  EXPECT_EQ(nullptr, retriever.get_shared("package://something"));
  EXPECT_EQ(nullptr, retriever.get_shared("service"));
  EXPECT_EQ(nullptr, retriever.get_shared("service://"));

  EXPECT_EQ(nullptr, retriever.get_shared("service://:"));
  EXPECT_EQ(nullptr, retriever.get_shared("service://:b"));
  EXPECT_EQ(nullptr, retriever.get_shared("service://a:b:c"));
}

class RosServiceResourceRetrieverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    client_node_ = rclcpp::Node::make_shared("test_client_node");
    server_node_ = rclcpp::Node::make_shared("test_server_node");
    executor_->add_node(client_node_);
    executor_->add_node(server_node_);

    executor_thread_ = std::make_unique<std::thread>(
      [executor = executor_.get()]() {executor->spin();});

    service_ = server_node_->create_service<GetResource>(
      "test_service",
      [&](const std::shared_ptr<GetResource::Request> request,
      std::shared_ptr<GetResource::Response> response) {
        *response = mock_service_function_.AsStdFunction()(request->path,
        request->etag);
      });

    retriever_ = std::make_unique<RosServiceResourceRetriever>(*client_node_);
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_thread_->join();

    retriever_.reset();
    service_.reset();
    server_node_.reset();
    client_node_.reset();
    executor_thread_.reset();
    executor_.reset();
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<std::thread> executor_thread_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::Node> server_node_;
  rclcpp::Service<GetResource>::SharedPtr service_;
  std::unique_ptr<RosServiceResourceRetriever> retriever_;

  MockFunction<GetResource::Response(const std::string &, const std::string &)>
  mock_service_function_;
};

TEST_F(RosServiceResourceRetrieverTest, SimpleE2EGet)
{
  std::vector<uint8_t> resource_data(2u, 3);
  GetResource::Response response;
  response.status_code = GetResource::Response::OK;
  response.body = resource_data;

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response));

  auto resource = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource);
  EXPECT_EQ(resource->data, resource_data);
}

TEST_F(RosServiceResourceRetrieverTest, ResourcePathExtraction)
{
  std::vector<uint8_t> resource_data(3u, 9);
  GetResource::Response response;
  response.status_code = GetResource::Response::OK;
  response.body = resource_data;

  // Expect the service to be called with the exact resource path after the ':'
  EXPECT_CALL(mock_service_function_, Call(std::string("foo/bar"), _)).WillOnce(Return(response));

  auto resource = retriever_->get_shared("service://test_service:foo/bar");
  ASSERT_NE(nullptr, resource);
  EXPECT_EQ(resource->data, resource_data);
}

TEST_F(RosServiceResourceRetrieverTest, ErrorStatusReturnsNull)
{
  GetResource::Response response;
  response.status_code = GetResource::Response::ERROR;

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response));

  auto resource = retriever_->get_shared("service://test_service:a");
  EXPECT_EQ(resource, nullptr);
}

TEST_F(RosServiceResourceRetrieverTest, UnknownStatusReturnsNull)
{
  GetResource::Response response;
  response.status_code = 17;

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response));

  auto resource = retriever_->get_shared("service://test_service:a");
  EXPECT_EQ(resource, nullptr);
}

TEST_F(RosServiceResourceRetrieverTest, DuplicateCallsWithoutEtagReturnCachedValue)
{
  std::vector<uint8_t> resource_data(2u, 3);
  GetResource::Response response1;
  response1.status_code = GetResource::Response::OK;
  response1.body = resource_data;
  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response1));

  auto resource1 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource1);
  EXPECT_EQ(resource1->data, resource_data);

  auto resource2 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource2);
  EXPECT_EQ(resource2->data, resource_data);

  EXPECT_EQ(resource1, resource2);
}

TEST_F(RosServiceResourceRetrieverTest, NotModifiedStatusReturnsCachedValue)
{
  std::vector<uint8_t> resource_data(2u, 3);
  GetResource::Response response1;
  response1.status_code = GetResource::Response::OK;
  response1.body = resource_data;
  response1.etag = "something";
  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response1));

  auto resource1 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource1);
  EXPECT_EQ(resource1->data, resource_data);

  GetResource::Response response2;
  response2.status_code = GetResource::Response::NOT_MODIFIED;
  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response2));

  auto resource2 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource2);
  EXPECT_EQ(resource2->data, resource_data);

  EXPECT_EQ(resource1, resource2);
}

TEST_F(RosServiceResourceRetrieverTest, NotModifiedReturnsEmptyCachedValue)
{
  GetResource::Response response;
  response.status_code = GetResource::Response::NOT_MODIFIED;
  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response));

  auto resource = retriever_->get_shared("service://test_service:a");
  EXPECT_EQ(resource, nullptr);
}

TEST_F(RosServiceResourceRetrieverTest, MultipleServices)
{
  MockFunction<GetResource::Response(const std::string &, const std::string &)>
  mock_service_function2;
  auto service2 = server_node_->create_service<GetResource>(
    "test_service2",
    [&](const std::shared_ptr<GetResource::Request> request,
    std::shared_ptr<GetResource::Response> response) {
      *response = mock_service_function2.AsStdFunction()(request->path,
      request->etag);
    });

  std::vector<uint8_t> resource_data1(2u, 3);
  GetResource::Response response1;
  response1.status_code = GetResource::Response::OK;
  response1.body = resource_data1;

  std::vector<uint8_t> resource_data2(2u, 8);
  GetResource::Response response2;
  response2.status_code = GetResource::Response::OK;
  response2.body = resource_data2;

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response1));

  auto resource1 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource1);
  EXPECT_EQ(resource1->data, resource_data1);

  EXPECT_CALL(mock_service_function2, Call(_, _)).WillOnce(Return(response2));

  auto resource2 = retriever_->get_shared("service://test_service2:a");
  ASSERT_NE(nullptr, resource2);
  EXPECT_EQ(resource2->data, resource_data2);
}

TEST_F(RosServiceResourceRetrieverTest, ServiceTimeoutViaParameterOverrideReturnsNull)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    std::string(RosServiceResourceRetriever::service_timeout_param_name), 50);
  auto custom_node = rclcpp::Node::make_shared("test_override_client_node", options);
  RosServiceResourceRetriever custom_retriever(*custom_node);

  GetResource::Response response;
  response.status_code = GetResource::Response::OK;
  response.body = std::vector<uint8_t>(2u, 3);

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(
    Invoke(
      [response](const std::string &, const std::string &) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        return response;
      }));

  auto resource = custom_retriever.get_shared("service://test_service:a");
  EXPECT_EQ(resource, nullptr);
}

TEST_F(RosServiceResourceRetrieverTest, RuntimeServiceTimeoutParameterUpdate)
{
  const std::string param_name(RosServiceResourceRetriever::service_timeout_param_name);

  // Set runtime timeout to 50ms -> 200ms service call should time out.
  auto set_result = client_node_->set_parameter(rclcpp::Parameter(param_name, 50));
  ASSERT_TRUE(set_result.successful);

  std::vector<uint8_t> resource_data(2u, 3);
  GetResource::Response response;
  response.status_code = GetResource::Response::OK;
  response.body = resource_data;

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(
    Invoke(
      [response](const std::string &, const std::string &) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        return response;
      }));

  EXPECT_EQ(nullptr, retriever_->get_shared("service://test_service:a"));

  // Update runtime timeout to 1000ms -> 100ms service call should now succeed.
  set_result = client_node_->set_parameter(rclcpp::Parameter(param_name, 1000));
  ASSERT_TRUE(set_result.successful);

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(
    Invoke(
      [response](const std::string &, const std::string &) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return response;
      }));

  auto resource = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource);
  EXPECT_EQ(resource->data, resource_data);
}

TEST_F(RosServiceResourceRetrieverTest, InvalidRuntimeServiceTimeoutParameterRejected)
{
  const std::string param_name(RosServiceResourceRetriever::service_timeout_param_name);

  ASSERT_TRUE(client_node_->set_parameter(rclcpp::Parameter(param_name, 500)).successful);

  EXPECT_FALSE(client_node_->set_parameter(rclcpp::Parameter(param_name, 0)).successful);
  EXPECT_FALSE(client_node_->set_parameter(rclcpp::Parameter(param_name, -50)).successful);
  EXPECT_FALSE(
    client_node_->set_parameter(
      rclcpp::Parameter(param_name, std::numeric_limits<int64_t>::max())).successful);
  EXPECT_FALSE(
    client_node_->set_parameter(rclcpp::Parameter(param_name, "invalid_timeout")).successful);

  // Previous valid timeout (500ms) remains intact.
  EXPECT_EQ(500, client_node_->get_parameter(param_name).as_int());
}

TEST_F(RosServiceResourceRetrieverTest, InvalidInitialServiceTimeoutParameterFallsBackToDefault)
{
  const std::string param_name(RosServiceResourceRetriever::service_timeout_param_name);
  const std::vector<rclcpp::ParameterValue> invalid_overrides = {
    rclcpp::ParameterValue(0),
    rclcpp::ParameterValue(-50),
    rclcpp::ParameterValue(std::numeric_limits<int64_t>::max()),
    rclcpp::ParameterValue(std::string("invalid_timeout")),
  };

  std::vector<uint8_t> resource_data(2u, 3);
  GetResource::Response response;
  response.status_code = GetResource::Response::OK;
  response.body = resource_data;

  for (size_t i = 0; i < invalid_overrides.size(); ++i) {
    rclcpp::NodeOptions options;
    options.append_parameter_override(param_name, invalid_overrides[i]);
    auto custom_node = rclcpp::Node::make_shared(
      "test_invalid_override_node_" + std::to_string(i), options);
    RosServiceResourceRetriever custom_retriever(*custom_node);

    EXPECT_EQ(
      RosServiceResourceRetriever::default_service_timeout.count(),
      custom_node->get_parameter(param_name).as_int());

    EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(
      Invoke(
        [response](const std::string &, const std::string &) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          return response;
        }));

    auto resource = custom_retriever.get_shared("service://test_service:a");
    ASSERT_NE(nullptr, resource);
    EXPECT_EQ(resource->data, resource_data);
  }
}

}  // namespace
}  // namespace resource_retriever_service_plugin

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
