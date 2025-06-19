# PID-Based Red Object Tracking with DJI Tello Drone

This project uses a **PID controller** to enable a DJI Tello drone to autonomously track a red object using **computer vision** with OpenCV and `djitellopy`.

![Tello PID Tracker](https://github.com/jayshiv06/PID-Based-Red-Object-Tracking-with-DJI-Tello-Drone/assets/your-image-link) <!-- Optional: add GIF or screenshot -->

---

## Features

- Real-time **object detection** using HSV color filtering
- **PID control** for:
  - Yaw alignment (horizontal)
  - Vertical positioning
  - Forward/backward movement (based on object area)
- Live camera feed visualization
- Matplotlib plot of PID outputs (for tuning and analysis)

---

## 🛠️ Tech Stack

- `Python`
- `OpenCV`
- `djitellopy`
- `NumPy`
- `Matplotlib`

---

## Installation

```bash
pip install opencv-python numpy matplotlib djitellopy
```

🧑‍💻 Author

Jayharish Shivakumar
📍 University of Southampton — Mechatronic Engineering <br>
📬 [LinkedIn](https://www.linkedin.com/in/jayharish-shivakumar-18591b275?lipi=urn%3Ali%3Apage%3Ad_flagship3_profile_view_base_contact_details%3BJ8gsKPwrTfK399uKGHno3w%3D%3D)
