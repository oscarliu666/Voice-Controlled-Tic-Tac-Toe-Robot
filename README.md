## Voice-Controlled Tic-Tac-Toe Robot

An interactive robotic system that plays Tic-Tac-Toe using voice commands and physical robotic manipulation.
The system integrates speech recognition, game logic, and a full robotic motion pipeline to translate spoken moves into precise, smooth actions executed by a 6-DOF robotic arm.

## Project Overview

This project demonstrates a multi-modal robotic system combining natural language input with real-time robotic control.
Voice commands are issued as discrete grid coordinates (e.g., “one one”, “one two”, “two three”), 
which are mapped to board positions and translated into precise robotic motion.


## Role & Contribution

Role: Robotics & Motion Control Developer (Team of 3)

Primary Ownership (My Contribution):

Designed and implemented the entire robotic manipulation pipeline

Implemented inverse kinematics, coordinate mapping, trajectory planning, and motion execution

Focused on motion stability, precision, and smoothness during physical interaction

Collaborative Components:

Speech recognition (Vosk)

High-level game logic and rule enforcement

In short: I owned all robotics, kinematics, and motion control components, while collaborating on speech recognition and game logic.

🔧 Technical Implementation (Robotics Focus)

Implemented geometric inverse kinematics to convert 2D board coordinates into 6-DOF joint-space targets

Designed coordinate frame transformations between game board space and robot workspace

Developed cubic joint-space trajectory generation for smooth, stable robotic motion

Implemented motion primitives for hovering, picking, and placement operations

Reduced mechanical jitter by ~60% through trajectory smoothing and motion optimization

## System Capabilities

Accurate mapping from spoken game commands → physical robot actions

Precise board-to-workspace coordinate transformation

Smooth and repeatable robotic motion with minimal mechanical jitter

Robust integration of speech input, logic control, and real-time motion execution

## Tech Stack

Robotics: SO-101 Robotic Arm

Control & Planning: Inverse Kinematics, Cubic Trajectory Planning

Programming: Python

Speech Recognition: Vosk

System Integration: Multi-module, real-time control pipeline

## Key Results

Achieved reliable voice-controlled gameplay with physical robot execution

Reduced motion jitter by ~60% compared to naive point-to-point motion

Successfully integrated perception, logic, and control into a unified robotic system

## Learning Outcomes

This project strengthened my understanding of:

Robotic kinematics and trajectory planning

Coordinating multiple processing pipelines in real-time systems

Translating high-level commands into safe, smooth, low-level robotic motion

## Repository Purpose

This repository is shared as part of my robotics and embedded systems portfolio and reflects my primary contributions to robotic manipulation, motion planning, and control.
## Poster
<img width="1226" height="320" alt="poster1" src="https://github.com/user-attachments/assets/5828078f-7e4e-4c52-8145-6bf3179c1223" />

<img width="1641" height="1025" alt="poster2" src="https://github.com/user-attachments/assets/d6853025-ba73-470a-b618-62fe383e3cfe" />
<img width="1557" height="415" alt="poster3" src="https://github.com/user-attachments/assets/84091d66-4ba4-4464-a2d4-2c7f5c7c8260" />
<img width="1634" height="462" alt="poster4" src="https://github.com/user-attachments/assets/90a5f7d9-9acd-461a-a12a-9ee6e23e119b" />
