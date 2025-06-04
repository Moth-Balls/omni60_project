/* Detects for green color objects and puts a bounding box around them */
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

struct BoundingBoxData {
    cv::Rect rect;
};

cv::Mat detect_green_object(cv::Mat& image, std::vector<BoundingBoxData>& bounding_boxes) {
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Define the lower and upper bounds for green color in HSV
    cv::Scalar lower_green(35, 40, 40); 
    cv::Scalar upper_green(90, 255, 255);

    // Create a binary mask that identifies green regions in the image
    cv::Mat green_mask;
    cv::inRange(hsv_image, lower_green, upper_green, green_mask);

    // Find contours in the binary mask
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(green_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Iterate through the contours and draw bounding boxes around the green objects
    for (size_t i = 0; i < contours.size(); i++) {

        cv::Rect bounding_box = cv::boundingRect(contours[i]);

        // Create a BoundingBoxData struct and populate it
        BoundingBoxData box_data;
        box_data.rect = bounding_box;
        bounding_boxes.push_back(box_data);

        // Draw the rectangle on the image
        cv::rectangle(image, bounding_box.tl(), bounding_box.br(), cv::Scalar(0, 0, 255), 2);
    }

    return image; 
}






