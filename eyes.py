import cv2
import numpy as np
import pygame
import time
import random
import RPi.GPIO as GPIO
from cvzone.FaceDetectionModule import FaceDetector

# Servo Setup
GPIO.setmode(GPIO.BCM)
servo1_pin = 17
servo2_pin = 27
GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)
pwm1 = GPIO.PWM(servo1_pin, 50)
pwm2 = GPIO.PWM(servo2_pin, 50)
pwm1.start(7.5)
pwm2.start(7.5)

def set_angle(pwm, angle):
    duty = 2.5 + (angle / 18.0)
    pwm.ChangeDutyCycle(duty)

# Pygame Setup
pygame.init()

# 3.5-inch Pi display resolution
WIDTH, HEIGHT = 480, 320
screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN)
pygame.display.set_caption("Cute Eyes + Servo")

BLACK = (0, 0, 0)
BLUE = (0, 170, 255)
DARK_BLUE = (0, 120, 200)
PINK = (255, 100, 150)

# Dynamically centered eye positions
eye_left = [WIDTH // 3, HEIGHT // 2]
eye_right = [2 * WIDTH // 3, HEIGHT // 2]

eye_size = WIDTH // 6
movement_range = WIDTH // 16  # Movement scaling

last_blink = time.time()
blink_duration = 0.1
blink_interval = random.randint(2, 5)
is_blinking = False

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
face_detector = FaceDetector()

running = True
while running:
    screen.fill(BLACK)
    success, img = cap.read()
    img, bboxs = face_detector.findFaces(img, draw=False)

    if bboxs:
        fx, fy = bboxs[0]["center"]
        target_eyeX = np.interp(fx, [100, 540], [movement_range, -movement_range])
        target_eyeY = np.interp(fy, [50, 430], [movement_range, -movement_range])

        eye_left[0] = WIDTH // 3 + target_eyeX
        eye_left[1] = HEIGHT // 2 + target_eyeY
        eye_right[0] = 2 * WIDTH // 3 + target_eyeX
        eye_right[1] = HEIGHT // 2 + target_eyeY

        # Servo control
        angle1 = int(np.interp(fx, [100, 540], [0, 180]))
        angle2 = int(np.interp(fy, [50, 430], [0, 180]))

        print(f"Servo 1 (X-Axis) Angle: {angle1}°, Servo 2 (Y-Axis) Angle: {angle2}°")

        # Uncomment to control servos
        # set_angle(pwm1, angle1)
        # set_angle(pwm2, angle2)

    # Blinking logic
    current_time = time.time()
    if current_time - last_blink >= blink_interval:
        is_blinking = True
        last_blink = current_time
        blink_interval = random.randint(2, 5)

    if is_blinking and current_time - last_blink >= blink_duration:
        is_blinking = False

    # Draw Eyes
    if not is_blinking:
        pygame.draw.rect(screen, PINK, (eye_left[0] - eye_size//2, eye_left[1] - eye_size//2, eye_size, eye_size), border_radius=30)
        pygame.draw.rect(screen, PINK, (eye_right[0] - eye_size//2, eye_right[1] - eye_size//2, eye_size, eye_size), border_radius=30)

        # Inner glow
        pygame.draw.rect(screen, BLUE, (eye_left[0] - eye_size//2 + 10, eye_left[1] - eye_size//2 + 10, eye_size - 20, eye_size - 20), border_radius=20)
        pygame.draw.rect(screen, BLUE, (eye_right[0] - eye_size//2 + 10, eye_right[1] - eye_size//2 + 10, eye_size - 20, eye_size - 20), border_radius=20)
    else:
        # Closed eyes
        pygame.draw.rect(screen, DARK_BLUE, (eye_left[0] - eye_size//2, eye_left[1] - 5, eye_size, 10), border_radius=5)
        pygame.draw.rect(screen, DARK_BLUE, (eye_right[0] - eye_size//2, eye_right[1] - 5, eye_size, 10), border_radius=5)

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

# Cleanup
pwm1.stop()
pwm2.stop()
GPIO.cleanup()
cap.release()
pygame.quit()
