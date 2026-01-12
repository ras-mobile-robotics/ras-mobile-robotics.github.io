---
layout: default
title: Resubmission
parent: Tutorials
sort: 5
---

# Resubmission Change Order

- **Assignment:** [e.g., Lab 2]
- **Date:** YYYY-MM-DD
- **Student Name:** [Your Name]
- **Original Submission Date:** YYYY-MM-DD

---

## 1. Failure Analysis (The "Why")
*Provide a technical explanation of why the previous submission failed to meet the requirements. Focus on the root cause, not just the symptoms.*

> **Example:** The robot failed to maintain a constant distance from the wall because the Proportional gain ($K_p$) was too high, causing undamped oscillations. Additionally, the ultrasonic sensor was being polled faster than its hardware refresh rate, leading to stale data.

**Root Cause:**
- [ ] Logic Error
- [ ] Hardware Configuration
- [ ] Parameter Tuning
- [ ] Environmental/Noise Issues

## 2. Implementation Delta (The "What")
*List the specific changes made to the code or hardware. If code-based, reference specific functions or files.*

* **Modified `pid_controller.py`:** Reduced K_p from 1.5 to 0.8 and introduced a 10ms sleep to match sensor frequency.
* **Hardware:** Re-mounted the sensor 5cm higher to avoid ground reflections.
* **Git Diff:** [Link to Pull Request or Commit Hash]

## 3. Performance Verification (The "Proof")
*How do you know it works now? Provide quantitative data or a link to a demonstration.*

* **Data/Logs:** Attached `telemetry_v2.csv` showing the error margin dropped from 15cm to 2cm.
* **Video Proof:** [Link to video]
* **Success Criteria:** The robot successfully completed 3 full laps without colliding with the wall.