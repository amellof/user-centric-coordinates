# user-centric-coordinates
User-centric coordinates for applications leveraging 3-axis accelerometer data

To properly use the raw sensors data in many different applications, it is highly important to convert such data into a resilient and unbiased device- and environment-independent coordinate system. The most common way of accomplishing this conversion involves adopting the so-called world-coordinate system. This coordinate system, although effective in some situations, might lead to a strong bias in which unwanted characteristics are not properly decoupled  from the problem. As such biases can impair data inference later on (for different applications), we set forth the objective of discussing the problems associated with the device coordinate system (in which the data is captured) and the commonly-used world coordinate system. In doing so, we design a robust user coordinate system aimed at eliminating possible biases associated with the device and its environment. The new coordinate system can be invaluable for the aforementioned applications allowing the design of more effective machine-learning solutions using less training data and device's resources when deploying embedded solutions.

This document is part of the supplemental material of the paper entitled "User-centric coordinates for applications leveraging 3-axis accelerometer data" and aims at describing how the ideas and formulae we have presented in the article can be implemented from a practical point of view. The complete reference of the article is:

A. Ferreira, G. Santos, A. Rocha and S. Goldenstein, "User-Centric Coordinates for Applications Leveraging 3-Axis Accelerometer Data," in IEEE Sensors Journal, vol. 17, no. 16, pp. 5231-5243, Aug.15, 15 2017.
doi: 10.1109/JSEN.2017.2723840
URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7968434&isnumber=7987843
