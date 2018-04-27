#define TABLE_LENGTH 500
#define TABLE_WIDTH 3
#define TABLE_DIFF_SPEEDS_SAME_ALTITUDE 5

float look_up_tab[TABLE_LENGTH][TABLE_WIDTH] = {{234.84,130.27,0},
		{234.84,130.92,47.5},
		{234.84,131.77,95},
		{234.84,132.58,142.5},
		{234.84,133.18,190},
		{242.87,129.54,0},
		{242.87,130.18,47.5},
		{242.87,131.02,95},
		{242.87,131.81,142.5},
		{242.87,132.41,190},
		{250.9,128.81,0},
		{250.9,129.45,47.5},
		{250.9,130.26,95},
		{250.9,131.05,142.5},
		{250.9,131.64,190},
		{258.93,128.08,0},
		{258.93,128.71,47.5},
		{258.93,129.51,95},
		{258.93,130.28,142.5},
		{258.93,130.86,190},
		{266.97,127.36,0},
		{266.97,127.97,47.5},
		{266.97,128.76,95},
		{266.97,129.52,142.5},
		{266.97,130.09,190},
		{275,126.63,0},
		{275,127.23,47.5},
		{275,128.01,95},
		{275,128.76,142.5},
		{275,129.31,190},
		{283.03,125.9,0},
		{283.03,126.5,47.5},
		{283.03,127.26,95},
		{283.03,127.99,142.5},
		{283.03,128.54,190},
		{291.06,125.18,0},
		{291.06,125.76,47.5},
		{291.06,126.51,95},
		{291.06,127.23,142.5},
		{291.06,127.77,190},
		{299.09,124.45,0},
		{299.09,125.02,47.5},
		{299.09,125.76,95},
		{299.09,126.46,142.5},
		{299.09,126.99,190},
		{307.13,123.72,0},
		{307.13,124.29,47.5},
		{307.13,125.01,95},
		{307.13,125.7,142.5},
		{307.13,126.22,190},
		{315.16,123,0},
		{315.16,123.55,47.5},
		{315.16,124.26,95},
		{315.16,124.93,142.5},
		{315.16,125.44,190},
		{323.19,122.24,0},
		{323.19,122.78,47.5},
		{323.19,123.47,95},
		{323.19,124.14,142.5},
		{323.19,124.64,190},
		{331.22,121.48,0},
		{331.22,122.01,47.5},
		{331.22,122.69,95},
		{331.22,123.35,142.5},
		{331.22,123.84,190},
		{339.25,120.72,0},
		{339.25,121.24,47.5},
		{339.25,121.91,95},
		{339.25,122.55,142.5},
		{339.25,123.03,190},
		{347.28,119.96,0},
		{347.28,120.47,47.5},
		{347.28,121.13,95},
		{347.28,121.76,142.5},
		{347.28,122.23,190},
		{355.32,119.2,0},
		{355.32,119.71,47.5},
		{355.32,120.35,95},
		{355.32,120.97,142.5},
		{355.32,121.43,190},
		{363.35,118.45,0},
		{363.35,118.94,47.5},
		{363.35,119.57,95},
		{363.35,120.17,142.5},
		{363.35,120.63,190},
		{371.38,117.69,0},
		{371.38,118.17,47.5},
		{371.38,118.79,95},
		{371.38,119.38,142.5},
		{371.38,119.82,190},
		{379.41,116.93,0},
		{379.41,117.4,47.5},
		{379.41,118.01,95},
		{379.41,118.59,142.5},
		{379.41,119.02,190},
		{387.44,116.17,0},
		{387.44,116.63,47.5},
		{387.44,117.23,95},
		{387.44,117.79,142.5},
		{387.44,118.22,190},
		{395.48,115.39,0},
		{395.48,115.84,47.5},
		{395.48,116.42,95},
		{395.48,116.98,142.5},
		{395.48,117.39,190},
		{403.51,114.6,0},
		{403.51,115.04,47.5},
		{403.51,115.61,95},
		{403.51,116.15,142.5},
		{403.51,116.56,190},
		{411.54,113.8,0},
		{411.54,114.24,47.5},
		{411.54,114.79,95},
		{411.54,115.33,142.5},
		{411.54,115.72,190},
		{419.57,113.01,0},
		{419.57,113.43,47.5},
		{419.57,113.98,95},
		{419.57,114.5,142.5},
		{419.57,114.89,190},
		{427.6,112.21,0},
		{427.6,112.63,47.5},
		{427.6,113.16,95},
		{427.6,113.67,142.5},
		{427.6,114.05,190},
		{435.64,111.42,0},
		{435.64,111.82,47.5},
		{435.64,112.35,95},
		{435.64,112.85,142.5},
		{435.64,113.22,190},
		{443.67,110.62,0},
		{443.67,111.02,47.5},
		{443.67,111.53,95},
		{443.67,112.02,142.5},
		{443.67,112.39,190},
		{451.7,109.83,0},
		{451.7,110.22,47.5},
		{451.7,110.72,95},
		{451.7,111.19,142.5},
		{451.7,111.55,190},
		{459.73,109.03,0},
		{459.73,109.41,47.5},
		{459.73,109.9,95},
		{459.73,110.37,142.5},
		{459.73,110.72,190},
		{467.76,108.21,0},
		{467.76,108.58,47.5},
		{467.76,109.06,95},
		{467.76,109.51,142.5},
		{467.76,109.85,190},
		{475.8,107.38,0},
		{475.8,107.74,47.5},
		{475.8,108.2,95},
		{475.8,108.64,142.5},
		{475.8,108.97,190},
		{483.83,106.54,0},
		{483.83,106.89,47.5},
		{483.83,107.35,95},
		{483.83,107.78,142.5},
		{483.83,108.1,190},
		{491.86,105.7,0},
		{491.86,106.05,47.5},
		{491.86,106.49,95},
		{491.86,106.92,142.5},
		{491.86,107.23,190},
		{499.89,104.87,0},
		{499.89,105.21,47.5},
		{499.89,105.64,95},
		{499.89,106.05,142.5},
		{499.89,106.36,190},
		{507.92,104.03,0},
		{507.92,104.36,47.5},
		{507.92,104.79,95},
		{507.92,105.19,142.5},
		{507.92,105.49,190},
		{515.96,103.19,0},
		{515.96,103.52,47.5},
		{515.96,103.93,95},
		{515.96,104.33,142.5},
		{515.96,104.62,190},
		{523.99,102.36,0},
		{523.99,102.67,47.5},
		{523.99,103.08,95},
		{523.99,103.46,142.5},
		{523.99,103.75,190},
		{532.02,101.5,0},
		{532.02,101.79,47.5},
		{532.02,102.17,95},
		{532.02,102.53,142.5},
		{532.02,102.8,190},
		{540.05,100.55,0},
		{540.05,100.84,47.5},
		{540.05,101.21,95},
		{540.05,101.57,142.5},
		{540.05,101.83,190},
		{548.08,99.611,0},
		{548.08,99.894,47.5},
		{548.08,100.26,95},
		{548.08,100.6,142.5},
		{548.08,100.86,190},
		{556.12,98.668,0},
		{556.12,98.945,47.5},
		{556.12,99.299,95},
		{556.12,99.636,142.5},
		{556.12,99.887,190},
		{564.15,97.726,0},
		{564.15,97.996,47.5},
		{564.15,98.342,95},
		{564.15,98.671,142.5},
		{564.15,98.917,190},
		{572.18,96.783,0},
		{572.18,97.046,47.5},
		{572.18,97.384,95},
		{572.18,97.706,142.5},
		{572.18,97.946,190},
		{580.21,95.84,0},
		{580.21,96.097,47.5},
		{580.21,96.427,95},
		{580.21,96.741,142.5},
		{580.21,96.975,190},
		{588.24,94.897,0},
		{588.24,95.148,47.5},
		{588.24,95.47,95},
		{588.24,95.777,142.5},
		{588.24,96.004,190},
		{596.28,93.954,0},
		{596.28,94.199,47.5},
		{596.28,94.513,95},
		{596.28,94.812,142.5},
		{596.28,95.034,190},
		{604.31,93.012,0},
		{604.31,93.25,47.5},
		{604.31,93.556,95},
		{604.31,93.847,142.5},
		{604.31,94.063,190},
		{612.34,92.069,0},
		{612.34,92.301,47.5},
		{612.34,92.599,95},
		{612.34,92.882,142.5},
		{612.34,93.092,190},
		{620.37,91.126,0},
		{620.37,91.352,47.5},
		{620.37,91.642,95},
		{620.37,91.917,142.5},
		{620.37,92.122,190},
		{628.4,90.183,0},
		{628.4,90.403,47.5},
		{628.4,90.684,95},
		{628.4,90.952,142.5},
		{628.4,91.151,190},
		{636.43,89.24,0},
		{636.43,89.454,47.5},
		{636.43,89.727,95},
		{636.43,89.987,142.5},
		{636.43,90.18,190},
		{644.47,88.298,0},
		{644.47,88.505,47.5},
		{644.47,88.77,95},
		{644.47,89.022,142.5},
		{644.47,89.21,190},
		{652.5,87.355,0},
		{652.5,87.556,47.5},
		{652.5,87.813,95},
		{652.5,88.057,142.5},
		{652.5,88.239,190},
		{660.53,86.412,0},
		{660.53,86.607,47.5},
		{660.53,86.856,95},
		{660.53,87.092,142.5},
		{660.53,87.268,190},
		{668.56,85.469,0},
		{668.56,85.658,47.5},
		{668.56,85.899,95},
		{668.56,86.128,142.5},
		{668.56,86.298,190},
		{676.59,84.527,0},
		{676.59,84.709,47.5},
		{676.59,84.941,95},
		{676.59,85.163,142.5},
		{676.59,85.327,190},
		{684.63,83.584,0},
		{684.63,83.76,47.5},
		{684.63,83.984,95},
		{684.63,84.198,142.5},
		{684.63,84.356,190},
		{692.66,82.641,0},
		{692.66,82.81,47.5},
		{692.66,83.027,95},
		{692.66,83.233,142.5},
		{692.66,83.386,190},
		{700.69,81.698,0},
		{700.69,81.861,47.5},
		{700.69,82.07,95},
		{700.69,82.268,142.5},
		{700.69,82.415,190},
		{708.72,80.755,0},
		{708.72,80.912,47.5},
		{708.72,81.113,95},
		{708.72,81.303,142.5},
		{708.72,81.444,190},
		{716.75,79.673,0},
		{716.75,79.813,47.5},
		{716.75,79.992,95},
		{716.75,80.162,142.5},
		{716.75,80.288,190},
		{724.79,78.486,0},
		{724.79,78.622,47.5},
		{724.79,78.795,95},
		{724.79,78.959,142.5},
		{724.79,79.082,190},
		{732.82,77.3,0},
		{732.82,77.43,47.5},
		{732.82,77.598,95},
		{732.82,77.757,142.5},
		{732.82,77.875,190},
		{740.85,76.113,0},
		{740.85,76.239,47.5},
		{740.85,76.401,95},
		{740.85,76.554,142.5},
		{740.85,76.668,190},
		{748.88,74.926,0},
		{748.88,75.048,47.5},
		{748.88,75.203,95},
		{748.88,75.351,142.5},
		{748.88,75.461,190},
		{756.91,73.739,0},
		{756.91,73.857,47.5},
		{756.91,74.006,95},
		{756.91,74.148,142.5},
		{756.91,74.254,190},
		{764.95,72.553,0},
		{764.95,72.665,47.5},
		{764.95,72.809,95},
		{764.95,72.946,142.5},
		{764.95,73.047,190},
		{772.98,71.366,0},
		{772.98,71.474,47.5},
		{772.98,71.612,95},
		{772.98,71.743,142.5},
		{772.98,71.84,190},
		{781.01,70.179,0},
		{781.01,70.283,47.5},
		{781.01,70.415,95},
		{781.01,70.54,142.5},
		{781.01,70.633,190},
		{789.04,68.993,0},
		{789.04,69.091,47.5},
		{789.04,69.218,95},
		{789.04,69.338,142.5},
		{789.04,69.427,190},
		{797.07,67.806,0},
		{797.07,67.9,47.5},
		{797.07,68.021,95},
		{797.07,68.135,142.5},
		{797.07,68.22,190},
		{805.11,66.619,0},
		{805.11,66.709,47.5},
		{805.11,66.823,95},
		{805.11,66.932,142.5},
		{805.11,67.013,190},
		{813.14,65.433,0},
		{813.14,65.518,47.5},
		{813.14,65.626,95},
		{813.14,65.729,142.5},
		{813.14,65.806,190},
		{821.17,64.246,0},
		{821.17,64.326,47.5},
		{821.17,64.429,95},
		{821.17,64.527,142.5},
		{821.17,64.599,190},
		{829.2,63.059,0},
		{829.2,63.135,47.5},
		{829.2,63.232,95},
		{829.2,63.324,142.5},
		{829.2,63.392,190},
		{837.23,61.872,0},
		{837.23,61.944,47.5},
		{837.23,62.035,95},
		{837.23,62.121,142.5},
		{837.23,62.185,190},
		{845.27,60.686,0},
		{845.27,60.753,47.5},
		{845.27,60.838,95},
		{845.27,60.919,142.5},
		{845.27,60.978,190},
		{853.3,59.433,0},
		{853.3,59.489,47.5},
		{853.3,59.56,95},
		{853.3,59.628,142.5},
		{853.3,59.678,190},
		{861.33,57.798,0},
		{861.33,57.85,47.5},
		{861.33,57.918,95},
		{861.33,57.982,142.5},
		{861.33,58.029,190},
		{869.36,56.163,0},
		{869.36,56.212,47.5},
		{869.36,56.276,95},
		{869.36,56.336,142.5},
		{869.36,56.381,190},
		{877.39,54.528,0},
		{877.39,54.574,47.5},
		{877.39,54.634,95},
		{877.39,54.69,142.5},
		{877.39,54.732,190},
		{885.43,52.893,0},
		{885.43,52.936,47.5},
		{885.43,52.992,95},
		{885.43,53.044,142.5},
		{885.43,53.083,190},
		{893.46,51.258,0},
		{893.46,51.298,47.5},
		{893.46,51.35,95},
		{893.46,51.398,142.5},
		{893.46,51.435,190},
		{901.49,49.623,0},
		{901.49,49.66,47.5},
		{901.49,49.708,95},
		{901.49,49.753,142.5},
		{901.49,49.786,190},
		{909.52,47.988,0},
		{909.52,48.022,47.5},
		{909.52,48.065,95},
		{909.52,48.107,142.5},
		{909.52,48.138,190},
		{917.55,46.353,0},
		{917.55,46.384,47.5},
		{917.55,46.423,95},
		{917.55,46.461,142.5},
		{917.55,46.489,190},
		{925.58,44.718,0},
		{925.58,44.746,47.5},
		{925.58,44.781,95},
		{925.58,44.815,142.5},
		{925.58,44.84,190},
		{933.62,43.083,0},
		{933.62,43.107,47.5},
		{933.62,43.139,95},
		{933.62,43.169,142.5},
		{933.62,43.192,190},
		{941.65,41.448,0},
		{941.65,41.469,47.5},
		{941.65,41.497,95},
		{941.65,41.524,142.5},
		{941.65,41.543,190},
		{949.68,39.812,0},
		{949.68,39.831,47.5},
		{949.68,39.855,95},
		{949.68,39.878,142.5},
		{949.68,39.895,190},
		{957.71,37.322,0},
		{957.71,37.335,47.5},
		{957.71,37.353,95},
		{957.71,37.369,142.5},
		{957.71,37.381,190},
		{965.74,34.626,0},
		{965.74,34.638,47.5},
		{965.74,34.654,95},
		{965.74,34.668,142.5},
		{965.74,34.679,190},
		{973.78,31.931,0},
		{973.78,31.942,47.5},
		{973.78,31.955,95},
		{973.78,31.967,142.5},
		{973.78,31.976,190},
		{981.81,29.236,0},
		{981.81,29.245,47.5},
		{981.81,29.255,95},
		{981.81,29.266,142.5},
		{981.81,29.273,190},
		{989.84,26.541,0},
		{989.84,26.548,47.5},
		{989.84,26.556,95},
		{989.84,26.564,142.5},
		{989.84,26.57,190},
		{997.87,23.846,0},
		{997.87,23.851,47.5},
		{997.87,23.857,95},
		{997.87,23.863,142.5},
		{997.87,23.868,190},
		{1005.9,21.151,0},
		{1005.9,21.154,47.5},
		{1005.9,21.158,95},
		{1005.9,21.162,142.5},
		{1005.9,21.165,190},
		{1013.9,17.663,0},
		{1013.9,17.664,47.5},
		{1013.9,17.666,95},
		{1013.9,17.668,142.5},
		{1013.9,17.669,190},
		{1022,12.471,0},
		{1022,12.472,47.5},
		{1022,12.472,95},
		{1022,12.473,142.5},
		{1022,12.473,190},
		{1030,0,0},
		{1030,0,47.5},
		{1030,0,95},
		{1030,0,142.5},
		{1030,0,190}};
