# ☀️ Low-Power Solar Tracking System for Remote Environmental Monitoring

This project presents a **cost-effective**, **energy-efficient**, and **autonomously powered** single-axis solar tracking system, designed to support **off-grid environmental sensor nodes** in outdoor deployments.

## 📌 Overview

The system leverages:
- A **single-axis tracking mechanism** driven by a DC motor with encoder feedback.
- **SunPosition astronomical algorithm** combined with LDR-based fine-tuning for dynamic solar angle adjustment.
- **Real-time energy monitoring** using INA219 sensors (solar panel input, logic circuit, and motor load).
- **LoRaWAN communication** via Arduino MKR WAN 1310, with real-time data upload to **The Things Network (TTN)**.
- **Low-power design** with deep sleep scheduling using a DS3231 RTC module.
- **Interactive visualization** using InfluxDB + Grafana on a Raspberry Pi backend.

## 📦 Hardware Components

| Component                    | Model / Type                        |
|-----------------------------|-------------------------------------|
| MCU                         | Arduino MKR WAN 1310                |
| Solar Panel                 | 3W 6V Polycrystalline (145×145mm)   |
| Battery                     | 3.7V 3300mAh 18650 Li-ion           |
| Charging Module             | Adafruit BQ24074                    |
| Motor                       | N20 6V DC motor with encoder        |
| INA219 Current Sensor       | x3 (motor / logic / solar)          |
| LDR Light Sensors           | x2                                  |
| RTC                         | DS3231                              |
| Voltage Regulator           | Pololu S7V8F5 (5V output)           |
| Enclosure                   | PLA / silicone sealed (3D printed)  |

## 🔧 Features

- ✅ **Single-axis rotation** covering ±90° azimuthal range.
- ✅ **Auto-homing on startup** using mechanical end-stop and encoder feedback.
- ✅ **Day/night detection** and dynamic sleep interval switching (10min daytime / 30min nighttime).
- ✅ **LoRa reconnect logic** for reliable long-term outdoor deployment.
- ✅ **Low-power design**: average sleep current < 12mA.
- ✅ **Field-proven**: 3 rounds of rooftop deployments with performance comparison against fixed panels.

## 📊 Results Summary

| Deployment Date | Weather     | Tracking Gain | Energy Self-Sufficiency |
|-----------------|-------------|---------------|--------------------------|
| July 1–4        | Mixed       | ❌ (unstable)  | ❌                       |
| July 28–30      | Cloudy      | ❌ (fixed > tracking) | ❌               |
| Aug 14–17       | Sunny       | ✅ (1.45× gain) | ✅ (>40 days est.)       |

> 📍 All systems deployed at UCL One Pool Street rooftop garden, South-facing, 25° tilt angle.

## 📡 Data Visualization

- All power, current, voltage, and light intensity data are **uploaded to TTN**.
- Data are **subscribed via MQTT** on Raspberry Pi and stored in **InfluxDB**.
- **Grafana dashboards** display real-time tracking status and energy flow.

## 🚀 Repository Structure

