
import cv2

import numpy as np


RED = (0, 0, 255)
YELLOW = (0, 255, 255)
GREEN = (0, 255, 0)

GREEN_LIMIT_LOW = np.array([50, 100, 100], dtype=np.uint8)
GREEN_LIMIT_HIGH = np.array([70, 255, 255], dtype=np.uint8)

YELLOW_LIMIT_LOW = np.array([20, 100, 100], dtype=np.uint8)
YELLOW_LIMIT_HIGH = np.array([40, 255, 255], dtype=np.uint8)

RED_LIMIT_LOW_1 = np.array([0, 100, 100], dtype=np.uint8)
RED_LIMIT_HIGH_1 = np.array([10, 255, 255], dtype=np.uint8)
RED_LIMIT_LOW_2 = np.array([170, 100, 100], dtype=np.uint8)
RED_LIMIT_HIGH_2 = np.array([179, 255, 255], dtype=np.uint8)

MAX_COLOR_SHIFT = 40.0


def adjust_tone_and_temperature(
    img: np.ndarray,
    tone: float = 50.0,
    temperature: float = 50.0,
) -> np.ndarray:
    tone_amount = (np.clip(tone, 0.0, 100.0) - 50.0) / 50.0
    temperature_amount = (np.clip(temperature, 0.0, 100.0) - 50.0) / 50.0
    channel_shift = np.array(
        [
            -temperature_amount + tone_amount * 0.5,
            -tone_amount,
            temperature_amount + tone_amount * 0.5,
        ],
        dtype=np.float32,
    ) * MAX_COLOR_SHIFT

    adjusted = img.astype(np.float32) + channel_shift
    return np.clip(adjusted, 0, 255).astype(np.uint8)


def detect_color(img, mask, color, minimum_percentage: float):
    pixel_count = cv2.countNonZero(mask)

    enough_color = pixel_count * 100 >= mask.size * minimum_percentage

    if enough_color:
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        for contour in contours:
            if cv2.contourArea(contour) < 300:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(
                img,
                (x, y),
                (x + w, y + h),
                color,
                3,
            )

    return img, enough_color


def pipeline(
    img: np.ndarray,
    minimum_percentage: float,
    tone: float = 50.0,
    temperature: float = 50.0,
):
    if img is None or img.size == 0:
        raise ValueError("Received an empty camera frame")

    img = adjust_tone_and_temperature(img, tone, temperature)
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(hsv_image, GREEN_LIMIT_LOW, GREEN_LIMIT_HIGH)
    mask_yellow = cv2.inRange(hsv_image, YELLOW_LIMIT_LOW, YELLOW_LIMIT_HIGH)

    mask_red_low = cv2.inRange(hsv_image, RED_LIMIT_LOW_1, RED_LIMIT_HIGH_1)
    mask_red_high = cv2.inRange(hsv_image, RED_LIMIT_LOW_2, RED_LIMIT_HIGH_2)
    mask_red = cv2.bitwise_or(mask_red_low, mask_red_high)

    img, green_found = detect_color(img, mask_green, GREEN, minimum_percentage)
    img, red_found = detect_color(img, mask_red, RED, minimum_percentage)
    img, yellow_found = detect_color(img, mask_yellow, YELLOW, minimum_percentage)

    return img, green_found, yellow_found, red_found
