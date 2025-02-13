# R2P2 Buck Converter for DC Microgrid

## Overview

The `BuckR2P2_Latin_F.m` script models a R2P2-buck converter using state-space representation and computes its transfer function. It generates plots of the open-loop frequency response and loop gain, as presented in the associated article. Additionally, it outputs parameter values corresponding to the tables in the article.

## Files

- `BuckR2P2_Latin_F.m`: MATLAB script containing the state-space model, transfer function computation, controller, frequency response analysis, and parameter value outputs.

## Requirements

- MATLAB R2017b or later.

## Usage

1. **Running the Script**:
   - Open `BuckR2P2_Latin_F.m` in MATLAB.
   - Execute the script by clicking the "Run" button or by typing `run('BuckR2P2_Latin.m')` in the Command Window.

2. **Outputs**:
   - **Plots**:
     - *Open-Loop Frequency Response*: Bode plot illustrating the system's open-loop behavior.
     - *Loop Gain*: Plot depicting the gain of the feedback loop.
     - *Pole-zero map: Plot illustring de poles and zeros from duty cicly to output voltage.
   - **Parameter Values**:
     - Displays calculated parameter values corresponding to the tables in the article.

## Notes

- For detailed explanations of the models and analyses, refer to the associated article.





