---
layout: default
title: Grading Policy
parent: Course
sort: 2
---

## 1. Course Policy Mastery & Iterative Engineering

Robotics is an inherently iterative discipline. High-performance systems are rarely "correct" on the first attempt; they are the result of rigorous testing, failure analysis, and refinement. To mirror professional engineering workflows, this course employs a **Mastery-Based Redemption Policy** that rewards deep debugging and technical persistence.

## 2. The Mastery Token System

Each student is allotted **2 Mastery Tokens** for the semester.

- **Purpose**: One token allows you to resubmit any lab or programming assignment (excluding the Final Project) to improve your grade.
- **The Feedback Loop**: You may only trigger a resubmission **after** you have received your initial grade and feedback from the teaching staff.
- **The Window**: Once your grade is posted, you have exactly **seven (7) calendar days** to submit your revised work.
- **Eligibility**: Tokens may only be applied to assignments that received an initial "Good Faith Effort" (defined as a submission that attempts to address the problem).

## 3. Late Submission Policy
To ensure the course moves at a professional pace, late submissions incur significant flat penalties. **Mastery Tokens cannot be used to waive late penalties.**

- **Up to 24 Hours Late**: -15% flat penalty.
- **24 to 48 Hours Late**: -30% flat penalty.
- **Beyond 48 Hours**: Submissions are not accepted for credit.

## 4. Grading & Point Recovery
We use a **50% Recovery Model**. This allows you to earn back half of the points between your initial effective grade (after any late penalties) and a perfect score ().

**The Formula**
$$Final\ Grade = Initial\ Grade + \frac{100 - Initial\ Grade}{2}$$

## 5. Resubmission Requirements
A resubmission will not be graded unless it includes the following three components in the repository

- **A. Technical Reflection** A brief document answering
  1. **Root Cause** What specifically caused the failure in the previous version? (e.g., sensor noise, PID windup, or logic race condition).
  2. **The Delta** What specific logic, parameters, or hardware configurations were changed?
  3. **Validation** How did you verify the new solution is robust?
- **B. Git Diff / Pull Request** For any submissions with code, a GitHub link to a diff highlighting the specific lines of code changed between the original and the new version.
- **C. Video Demonstration** For any hardware-based lab, an unedited 20–60 second video demonstrating the robot’s successful performance is mandatory.


## 6. Case Study Examples

| Scenario | Raw Score | Timing | Effective Initial | Resubmission? | Final Grade |
| --- | --- | --- | --- | --- | --- |
| **The Precise Fix** | 70% | On Time | 70% | **Yes** | **85%** |
| **The Late Penalty** | 90% | 12h Late | 75% | **No** | **75%** |
| **Late + Mastery** | 90% | 12h Late | 75% | **Yes** | **87.5%** |
| **Hardware Crisis** | 80% | 40h Late | 50% | **Yes** | **75%** |

```note
**Note on Late + Mastery** If you submit 1 day late and get a 90%, your grade is a 75%. If you then use a token to resubmit and get 100% on the fix, your final grade is.

```
## 7. Logistics
To use a token, navigate to *Modules* on Canvas. Under *Lab Administration*, you will find the **Regrade Request - Master Token #1** and **Regrade Request - Master Token #2** forms.

Each form represents a single redemption. To apply a token to an assignment:
1. **Open the Form**: Click on the available *Regrade Request* forms.
2. **Complete the Template**: Use the Resubmission Change Order template (found in the forms themselves when opened, or can be viewed [here](../tutorials/resubmission)).
3. **Submit**: Click on *Start Assignment* on the top right on Canvas, upload your completed document, and submit.

```note
You must submit your request within seven (7) calendar days of receiving your initial grade.
```
