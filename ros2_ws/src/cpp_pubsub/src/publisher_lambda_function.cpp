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

const std::string MASTODON_INSTANCE = "https://mastodon.social";
const std::string ACCESS_TOKEN = "ACCESS_TOKEN";

size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

std::string fetchMastodonPosts() {
    CURL *curl = curl_easy_init();
    if (!curl) return "";

    std::string url = MASTODON_INSTANCE + "/api/v1/timelines/tag/activityros6";
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

class PriusController : public rclcpp::Node {
public:
    PriusController() : Node("prius_controller") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(5s, std::bind(&PriusController::checkMastodon, this));
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
        msg.angular.z = 0.0;

        if (command.find("forward") != std::string::npos) {
            msg.linear.x = 3.0;
        } else if (command.find("backward") != std::string::npos) {
            msg.linear.x = -3.0;
        } else if (command.find("left") != std::string::npos) {
            msg.angular.z = 1.5;
        } else if (command.find("right") != std::string::npos) {
            msg.angular.z = -1.5;
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
    rclcpp::spin(std::make_shared<PriusController>());
    rclcpp::shutdown();
    return 0;
}
