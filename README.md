# Injury Prediction and Performance Optimization Using Pose Estimation

This project aims to leverage OpenCap's open-source motion capture software, in combination with OpenPose and OpenSim models, to gather biomechanical data and utilize machine learning to predict significant factors in athletic motions. This approach helps optimize training, detect asymmetries, and prevent overtraining-related injuries in athletes.

## **Table of Contents**
- [Overview](#overview)
- [Features](#features)
- [Project Workflow](#project-workflow)
- [Installation](#installation)
- [Usage](#usage)
- [Results](#results)
- [Contributing](#contributing)
- [License](#license)

## **Overview**

Athletes' performance and injury prevention are crucial aspects that require a deep understanding of biomechanics. This project combines pose estimation, motion capture, and machine learning to analyze athletic movements and predict factors that contribute to injury risk and optimal performance. By identifying significant biomechanical markers through AI and machine learning models, this research aims to provide data-driven insights that can help in tailoring training regimens to optimize athletic motions and reduce injury risk.

## **Features**

- **Motion Capture**: Uses OpenCap and OpenPose to gather and synchronize videos, generating 3D skeleton models for biomechanical analysis.
- **Kinetic Analysis**: Utilizes OpenSim models to analyze kinetic data and extract significant biomechanical factors.
- **Machine Learning Analysis**: Applies Convolutional Neural Network (CNN) and XGBoost stacking ensemble models to predict key injury risk factors and optimize performance.
- **Data Analysis**: Tracks asymmetries, irregularities, and performance differences over time, providing insights into training improvements and injury prevention.

## **Project Workflow**

1. **Data Collection with OpenCap and OpenPose**:
   - OpenCap is used to capture synchronized video recordings of athletic movements, utilizing OpenPose for pose estimation.
   - The videos are processed to produce 3D skeleton models, which provide detailed motion data in collaboration with OpenSim models.

2. **Biomechanical Data Extraction**:
   - Run `kinetics.py` with appropriate video and time parameters to solve the optimal control problem and extract biomechanical data.
   - This step generates CSV files containing detailed biomechanical information such as ground reaction forces, joint torques, angles, and velocities.

3. **Machine Learning Analysis**:
   - Use `ML-Analysis.ipynb` for data preprocessing and feature engineering on the generated CSV data.
   - Apply Convolutional Neural Network (CNN) and XGBoost stacking ensemble models to predict significant factors in athletic motion, identify asymmetries, and assess injury risk.

4. **Performance and Injury Risk Assessment**:
   - Evaluate the ML models' predictions to identify patterns, irregularities, and factors contributing to overtraining or potential injuries.
   - Tailor training programs to optimize movements and prevent injuries.



## **Installation**

1. **Clone the repository with submodules**:
   ```bash
   git clone --recurse-submodules https://github.com/TheDanGriff/Injury-Prediction.git
2. cd Injury-Prediction
3. pip install -r requirements.txt
4. git submodule update --init --recursive

Usage
Collect Motion Data with OpenCap:

Follow OpenCap’s instructions to capture video data of athletic movements.
Run Kinetic Analysis:

Use kinetics.py with the appropriate video and time parameters to generate biomechanical data in CSV format

Machine Learning Analysis:

Open the ML-Analysis.ipynb notebook in the notebooks/ directory.
Follow the steps outlined in the notebook to preprocess the data, train the CNN/XGBoost models, and analyze significant factors affecting athletic motion.
Evaluate Results:

Use the generated CSV files and the ML predictions to assess asymmetries, irregularities, and potential injury risks.

Contributing
We welcome contributions! Here's how you can contribute:

Fork the repository.
Create a new branch:
bash
Copy code
git checkout -b feature-branch
Make your changes and commit:
bash
Copy code
git commit -m "Added a new feature"
Push to your branch:
bash
Copy code
git push origin feature-branch
Open a Pull Request.

License
This project is licensed under the MIT License - see the LICENSE file for details.
