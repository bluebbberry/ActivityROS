// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <curl/curl.h>
#include <json/json.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

// Mastodon API Credentials (Replace with your actual values)
const std::string MASTODON_INSTANCE = "https://mastodon.social";  // Change to your instance
const std::string ACCESS_TOKEN = "API-TOKEN";  // Replace with your API key

// Function to handle HTTP GET request to Mastodon
size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

std::string fetchMastodonPosts() {
    CURL *curl = curl_easy_init();
    if (!curl) return "";

    std::string url = MASTODON_INSTANCE + "/api/v1/timelines/tag/activityros3";  // Listening for #activityros posts
    std::string response;

    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, ("Authorization: Bearer " + ACCESS_TOKEN).c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "CURL Error: " << curl_easy_strerror(res) << std::endl;
    }

    curl_easy_cleanup(curl);
    return response;
}

// Function to parse Mastodon response and extract the latest command
std::string parseMastodonResponse(const std::string &response) {
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::istringstream responseStream(response);
    std::string errors;

    if (!Json::parseFromStream(reader, responseStream, &root, &errors)) {
        std::cerr << "JSON Parse Error: " << errors << std::endl;
        return "";
    }

    if (root.isArray() && !root.empty()) {
        std::string content = root[0]["content"].asString();
        return content;
    }

    return "";
}

// TurtleBot Controller Node
class TurtleController : public rclcpp::Node {
public:
    TurtleController() : Node("turtle_controller") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer to check Mastodon every 5 seconds
        timer_ = this->create_wall_timer(5s, std::bind(&TurtleController::checkMastodon, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    void checkMastodon() {
        std::string response = fetchMastodonPosts();
        std::string command = parseMastodonResponse(response);

        if (!command.empty()) {
            processCommand(command);
        }
    }

    void processCommand(const std::string &command) {
        auto msg = geometry_msgs::msg::Twist();

        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        if (command.find("forward") != std::string::npos) {
            msg.linear.x = 2.0;  // Move forward
        } else if (command.find("backward") != std::string::npos) {
            msg.linear.x = -2.0;  // Move backward
        } else if (command.find("left") != std::string::npos) {
            msg.angular.z = 2.0;  // Rotate left
        } else if (command.find("right") != std::string::npos) {
            msg.angular.z = -2.0; // Rotate right
        } else {
            RCLCPP_INFO(this->get_logger(), "No valid command found.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %.2f, angular.z = %.2f", msg.linear.x, msg.angular.z);
        publisher_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleController>());
    rclcpp::shutdown();
    return 0;
}
