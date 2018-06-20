#ifndef SHURIKEN_LOOKUP_TABLE 
#define SHURIKEN_LOOKUP_TABLE 

#define TABLE_LENGTH 500
#define TABLE_WIDTH 3 
#define TABLE_DIFF_SPEEDS_SAME_ALTITUDE 5
static const float32_t SimData[TABLE_LENGTH][TABLE_WIDTH] = {
{ 1135.9103, 215.2359, 0 },
{ 1135.9103, 217.074, 43 },
{ 1135.9103, 219.4926, 86 },
{ 1135.9103, 221.9749, 129 },
{ 1135.9103, 224.0651, 172 },
{ 1155.2244, 213.8622, 0 },
{ 1155.2244, 215.6699, 43 },
{ 1155.2244, 218.0483, 86 },
{ 1155.2244, 220.4892, 129 },
{ 1155.2244, 222.5443, 172 },
{ 1174.5383, 212.4884, 0 },
{ 1174.5383, 214.2658, 43 },
{ 1174.5383, 216.6041, 86 },
{ 1174.5383, 219.0034, 129 },
{ 1174.5383, 221.0236, 172 },
{ 1193.8524, 211.1147, 0 },
{ 1193.8524, 212.8617, 43 },
{ 1193.8524, 215.1598, 86 },
{ 1193.8524, 217.5177, 129 },
{ 1193.8524, 219.5028, 172 },
{ 1213.1664, 209.741, 0 },
{ 1213.1664, 211.4576, 43 },
{ 1213.1664, 213.7155, 86 },
{ 1213.1664, 216.0319, 129 },
{ 1213.1664, 217.982, 172 },
{ 1232.4805, 208.3672, 0 },
{ 1232.4805, 210.0535, 43 },
{ 1232.4805, 212.2712, 86 },
{ 1232.4805, 214.5462, 129 },
{ 1232.4805, 216.4612, 172 },
{ 1251.7946, 206.9935, 0 },
{ 1251.7946, 208.6494, 43 },
{ 1251.7946, 210.827, 86 },
{ 1251.7946, 213.0604, 129 },
{ 1251.7946, 214.9405, 172 },
{ 1271.1085, 205.6197, 0 },
{ 1271.1085, 207.2453, 43 },
{ 1271.1085, 209.3827, 86 },
{ 1271.1085, 211.5746, 129 },
{ 1271.1085, 213.4197, 172 },
{ 1290.4226, 204.246, 0 },
{ 1290.4226, 205.8412, 43 },
{ 1290.4226, 207.9384, 86 },
{ 1290.4226, 210.0889, 129 },
{ 1290.4226, 211.8989, 172 },
{ 1309.7366, 202.8722, 0 },
{ 1309.7366, 204.4371, 43 },
{ 1309.7366, 206.4941, 86 },
{ 1309.7366, 208.6031, 129 },
{ 1309.7366, 210.3782, 172 },
{ 1329.0507, 201.4985, 0 },
{ 1329.0507, 203.033, 43 },
{ 1329.0507, 205.0499, 86 },
{ 1329.0507, 207.1174, 129 },
{ 1329.0507, 208.8574, 172 },
{ 1348.3647, 200.1247, 0 },
{ 1348.3647, 201.6289, 43 },
{ 1348.3647, 203.6056, 86 },
{ 1348.3647, 205.6316, 129 },
{ 1348.3647, 207.3366, 172 },
{ 1367.6787, 198.751, 0 },
{ 1367.6787, 200.2248, 43 },
{ 1367.6787, 202.1613, 86 },
{ 1367.6787, 204.1459, 129 },
{ 1367.6787, 205.8159, 172 },
{ 1386.9928, 197.3772, 0 },
{ 1386.9928, 198.8207, 43 },
{ 1386.9928, 200.717, 86 },
{ 1386.9928, 202.6601, 129 },
{ 1386.9928, 204.2951, 172 },
{ 1406.3068, 196.0035, 0 },
{ 1406.3068, 197.4167, 43 },
{ 1406.3068, 199.2728, 86 },
{ 1406.3068, 201.1744, 129 },
{ 1406.3068, 202.7743, 172 },
{ 1425.6208, 194.6297, 0 },
{ 1425.6208, 196.0125, 43 },
{ 1425.6208, 197.8285, 86 },
{ 1425.6208, 199.6886, 129 },
{ 1425.6208, 201.2535, 172 },
{ 1444.9348, 193.256, 0 },
{ 1444.9348, 194.6085, 43 },
{ 1444.9348, 196.3842, 86 },
{ 1444.9348, 198.2029, 129 },
{ 1444.9348, 199.7328, 172 },
{ 1464.2489, 191.8822, 0 },
{ 1464.2489, 193.2044, 43 },
{ 1464.2489, 194.94, 86 },
{ 1464.2489, 196.7171, 129 },
{ 1464.2489, 198.212, 172 },
{ 1483.563, 190.5085, 0 },
{ 1483.563, 191.8003, 43 },
{ 1483.563, 193.4957, 86 },
{ 1483.563, 195.2314, 129 },
{ 1483.563, 196.6912, 172 },
{ 1502.877, 189.1347, 0 },
{ 1502.877, 190.3962, 43 },
{ 1502.877, 192.0514, 86 },
{ 1502.877, 193.7456, 129 },
{ 1502.877, 195.1705, 172 },
{ 1522.191, 187.761, 0 },
{ 1522.191, 188.9921, 43 },
{ 1522.191, 190.6071, 86 },
{ 1522.191, 192.2598, 129 },
{ 1522.191, 193.6497, 172 },
{ 1541.5051, 186.3872, 0 },
{ 1541.5051, 187.588, 43 },
{ 1541.5051, 189.1628, 86 },
{ 1541.5051, 190.7741, 129 },
{ 1541.5051, 192.1289, 172 },
{ 1560.8191, 185.0135, 0 },
{ 1560.8191, 186.1839, 43 },
{ 1560.8191, 187.7186, 86 },
{ 1560.8191, 189.2883, 129 },
{ 1560.8191, 190.6082, 172 },
{ 1580.1331, 183.6397, 0 },
{ 1580.1331, 184.7798, 43 },
{ 1580.1331, 186.2743, 86 },
{ 1580.1331, 187.8026, 129 },
{ 1580.1331, 189.0874, 172 },
{ 1599.4471, 182.2072, 0 },
{ 1599.4471, 183.3251, 43 },
{ 1599.4471, 184.7894, 86 },
{ 1599.4471, 186.2853, 129 },
{ 1599.4471, 187.542, 172 },
{ 1618.7612, 180.7358, 0 },
{ 1618.7612, 181.8303, 43 },
{ 1618.7612, 183.2638, 86 },
{ 1618.7612, 184.7284, 129 },
{ 1618.7612, 185.9587, 172 },
{ 1638.0752, 179.2644, 0 },
{ 1638.0752, 180.3355, 43 },
{ 1638.0752, 181.7383, 86 },
{ 1638.0752, 183.1715, 129 },
{ 1638.0752, 184.3755, 172 },
{ 1657.3893, 177.793, 0 },
{ 1657.3893, 178.8406, 43 },
{ 1657.3893, 180.2128, 86 },
{ 1657.3893, 181.6146, 129 },
{ 1657.3893, 182.7922, 172 },
{ 1676.7034, 176.3216, 0 },
{ 1676.7034, 177.3458, 43 },
{ 1676.7034, 178.6873, 86 },
{ 1676.7034, 180.0576, 129 },
{ 1676.7034, 181.209, 172 },
{ 1696.0173, 174.8501, 0 },
{ 1696.0173, 175.851, 43 },
{ 1696.0173, 177.1618, 86 },
{ 1696.0173, 178.5007, 129 },
{ 1696.0173, 179.6257, 172 },
{ 1715.3314, 173.3787, 0 },
{ 1715.3314, 174.3562, 43 },
{ 1715.3314, 175.6363, 86 },
{ 1715.3314, 176.9438, 129 },
{ 1715.3314, 178.0425, 172 },
{ 1734.6455, 171.9073, 0 },
{ 1734.6455, 172.8614, 43 },
{ 1734.6455, 174.1107, 86 },
{ 1734.6455, 175.3869, 129 },
{ 1734.6455, 176.4592, 172 },
{ 1753.9595, 170.4359, 0 },
{ 1753.9595, 171.3666, 43 },
{ 1753.9595, 172.5852, 86 },
{ 1753.9595, 173.83, 129 },
{ 1753.9595, 174.876, 172 },
{ 1773.2734, 168.9645, 0 },
{ 1773.2734, 169.8718, 43 },
{ 1773.2734, 171.0597, 86 },
{ 1773.2734, 172.273, 129 },
{ 1773.2734, 173.2928, 172 },
{ 1792.5876, 167.493, 0 },
{ 1792.5876, 168.3769, 43 },
{ 1792.5876, 169.5342, 86 },
{ 1792.5876, 170.7161, 129 },
{ 1792.5876, 171.7095, 172 },
{ 1811.9016, 166.0216, 0 },
{ 1811.9016, 166.8821, 43 },
{ 1811.9016, 168.0087, 86 },
{ 1811.9016, 169.1592, 129 },
{ 1811.9016, 170.1263, 172 },
{ 1831.2156, 164.5502, 0 },
{ 1831.2156, 165.3873, 43 },
{ 1831.2156, 166.4832, 86 },
{ 1831.2156, 167.6023, 129 },
{ 1831.2156, 168.543, 172 },
{ 1850.5297, 163.0788, 0 },
{ 1850.5297, 163.8925, 43 },
{ 1850.5297, 164.9576, 86 },
{ 1850.5297, 166.0453, 129 },
{ 1850.5297, 166.9598, 172 },
{ 1869.8438, 161.6074, 0 },
{ 1869.8438, 162.3977, 43 },
{ 1869.8438, 163.4321, 86 },
{ 1869.8438, 164.4884, 129 },
{ 1869.8438, 165.3765, 172 },
{ 1889.1577, 160.1359, 0 },
{ 1889.1577, 160.9029, 43 },
{ 1889.1577, 161.9066, 86 },
{ 1889.1577, 162.9315, 129 },
{ 1889.1577, 163.7933, 172 },
{ 1908.4717, 158.6645, 0 },
{ 1908.4717, 159.4081, 43 },
{ 1908.4717, 160.3811, 86 },
{ 1908.4717, 161.3746, 129 },
{ 1908.4717, 162.21, 172 },
{ 1927.7859, 157.1931, 0 },
{ 1927.7859, 157.9133, 43 },
{ 1927.7859, 158.8556, 86 },
{ 1927.7859, 159.8176, 129 },
{ 1927.7859, 160.6268, 172 },
{ 1947.0999, 155.7217, 0 },
{ 1947.0999, 156.4184, 43 },
{ 1947.0999, 157.3301, 86 },
{ 1947.0999, 158.2607, 129 },
{ 1947.0999, 159.0435, 172 },
{ 1966.4138, 154.2503, 0 },
{ 1966.4138, 154.9236, 43 },
{ 1966.4138, 155.8046, 86 },
{ 1966.4138, 156.7038, 129 },
{ 1966.4138, 157.4603, 172 },
{ 1985.7279, 152.6483, 0 },
{ 1985.7279, 153.3079, 43 },
{ 1985.7279, 154.1702, 86 },
{ 1985.7279, 155.0497, 129 },
{ 1985.7279, 155.7889, 172 },
{ 2005.042, 151.0147, 0 },
{ 2005.042, 151.6562, 43 },
{ 2005.042, 152.4947, 86 },
{ 2005.042, 153.3499, 129 },
{ 2005.042, 154.0688, 172 },
{ 2024.356, 149.3812, 0 },
{ 2024.356, 150.0045, 43 },
{ 2024.356, 150.8192, 86 },
{ 2024.356, 151.6501, 129 },
{ 2024.356, 152.3486, 172 },
{ 2043.67, 147.7476, 0 },
{ 2043.67, 148.3528, 43 },
{ 2043.67, 149.1437, 86 },
{ 2043.67, 149.9503, 129 },
{ 2043.67, 150.6284, 172 },
{ 2062.9841, 146.114, 0 },
{ 2062.9841, 146.701, 43 },
{ 2062.9841, 147.4682, 86 },
{ 2062.9841, 148.2505, 129 },
{ 2062.9841, 148.9083, 172 },
{ 2082.2981, 144.4805, 0 },
{ 2082.2981, 145.0493, 43 },
{ 2082.2981, 145.7927, 86 },
{ 2082.2981, 146.5508, 129 },
{ 2082.2981, 147.1881, 172 },
{ 2101.6121, 142.8469, 0 },
{ 2101.6121, 143.3976, 43 },
{ 2101.6121, 144.1172, 86 },
{ 2101.6121, 144.851, 129 },
{ 2101.6121, 145.468, 172 },
{ 2120.9263, 141.2133, 0 },
{ 2120.9263, 141.7459, 43 },
{ 2120.9263, 142.4417, 86 },
{ 2120.9263, 143.1512, 129 },
{ 2120.9263, 143.7478, 172 },
{ 2140.2402, 139.5798, 0 },
{ 2140.2402, 140.0941, 43 },
{ 2140.2402, 140.7662, 86 },
{ 2140.2402, 141.4514, 129 },
{ 2140.2402, 142.0276, 172 },
{ 2159.5542, 137.9462, 0 },
{ 2159.5542, 138.4424, 43 },
{ 2159.5542, 139.0907, 86 },
{ 2159.5542, 139.7516, 129 },
{ 2159.5542, 140.3074, 172 },
{ 2178.8682, 136.3127, 0 },
{ 2178.8682, 136.7907, 43 },
{ 2178.8682, 137.4152, 86 },
{ 2178.8682, 138.0518, 129 },
{ 2178.8682, 138.5873, 172 },
{ 2198.1824, 134.6791, 0 },
{ 2198.1824, 135.139, 43 },
{ 2198.1824, 135.7397, 86 },
{ 2198.1824, 136.3521, 129 },
{ 2198.1824, 136.8671, 172 },
{ 2217.4963, 133.0455, 0 },
{ 2217.4963, 133.4873, 43 },
{ 2217.4963, 134.0642, 86 },
{ 2217.4963, 134.6523, 129 },
{ 2217.4963, 135.147, 172 },
{ 2236.8105, 131.412, 0 },
{ 2236.8105, 131.8355, 43 },
{ 2236.8105, 132.3887, 86 },
{ 2236.8105, 132.9525, 129 },
{ 2236.8105, 133.4268, 172 },
{ 2256.1245, 129.7784, 0 },
{ 2256.1245, 130.1838, 43 },
{ 2256.1245, 130.7132, 86 },
{ 2256.1245, 131.2527, 129 },
{ 2256.1245, 131.7066, 172 },
{ 2275.4385, 128.1449, 0 },
{ 2275.4385, 128.5321, 43 },
{ 2275.4385, 129.0377, 86 },
{ 2275.4385, 129.5529, 129 },
{ 2275.4385, 129.9865, 172 },
{ 2294.7524, 126.4351, 0 },
{ 2294.7524, 126.8064, 43 },
{ 2294.7524, 127.2911, 86 },
{ 2294.7524, 127.7848, 129 },
{ 2294.7524, 128.2, 172 },
{ 2314.0667, 124.5466, 0 },
{ 2314.0667, 124.9045, 43 },
{ 2314.0667, 125.3715, 86 },
{ 2314.0667, 125.847, 129 },
{ 2314.0667, 126.2469, 172 },
{ 2333.3806, 122.6581, 0 },
{ 2333.3806, 123.0025, 43 },
{ 2333.3806, 123.4518, 86 },
{ 2333.3806, 123.9092, 129 },
{ 2333.3806, 124.2939, 172 },
{ 2352.6948, 120.7697, 0 },
{ 2352.6948, 121.1006, 43 },
{ 2352.6948, 121.5322, 86 },
{ 2352.6948, 121.9714, 129 },
{ 2352.6948, 122.3408, 172 },
{ 2372.0088, 118.8812, 0 },
{ 2372.0088, 119.1986, 43 },
{ 2372.0088, 119.6125, 86 },
{ 2372.0088, 120.0337, 129 },
{ 2372.0088, 120.3877, 172 },
{ 2391.3228, 116.9928, 0 },
{ 2391.3228, 117.2967, 43 },
{ 2391.3228, 117.6929, 86 },
{ 2391.3228, 118.0959, 129 },
{ 2391.3228, 118.4347, 172 },
{ 2410.6367, 115.1043, 0 },
{ 2410.6367, 115.3947, 43 },
{ 2410.6367, 115.7733, 86 },
{ 2410.6367, 116.1582, 129 },
{ 2410.6367, 116.4816, 172 },
{ 2429.9507, 113.2159, 0 },
{ 2429.9507, 113.4928, 43 },
{ 2429.9507, 113.8536, 86 },
{ 2429.9507, 114.2204, 129 },
{ 2429.9507, 114.5286, 172 },
{ 2449.2649, 111.3274, 0 },
{ 2449.2649, 111.5908, 43 },
{ 2449.2649, 111.934, 86 },
{ 2449.2649, 112.2826, 129 },
{ 2449.2649, 112.5755, 172 },
{ 2468.5789, 109.439, 0 },
{ 2468.5789, 109.6889, 43 },
{ 2468.5789, 110.0143, 86 },
{ 2468.5789, 110.3449, 129 },
{ 2468.5789, 110.6224, 172 },
{ 2487.8931, 107.5505, 0 },
{ 2487.8931, 107.7869, 43 },
{ 2487.8931, 108.0947, 86 },
{ 2487.8931, 108.4071, 129 },
{ 2487.8931, 108.6693, 172 },
{ 2507.207, 105.662, 0 },
{ 2507.207, 105.885, 43 },
{ 2507.207, 106.1751, 86 },
{ 2507.207, 106.4693, 129 },
{ 2507.207, 106.7163, 172 },
{ 2526.521, 103.7736, 0 },
{ 2526.521, 103.9831, 43 },
{ 2526.521, 104.2554, 86 },
{ 2526.521, 104.5315, 129 },
{ 2526.521, 104.7632, 172 },
{ 2545.835, 101.8852, 0 },
{ 2545.835, 102.0811, 43 },
{ 2545.835, 102.3358, 86 },
{ 2545.835, 102.5938, 129 },
{ 2545.835, 102.8071, 172 },
{ 2565.1489, 99.6709, 0 },
{ 2565.1489, 99.8453, 43 },
{ 2565.1489, 100.0723, 86 },
{ 2565.1489, 100.3027, 129 },
{ 2565.1489, 100.4961, 172 },
{ 2584.4631, 97.4042, 0 },
{ 2584.4631, 97.5693, 43 },
{ 2584.4631, 97.7841, 86 },
{ 2584.4631, 98.0021, 129 },
{ 2584.4631, 98.1852, 172 },
{ 2603.7771, 95.1375, 0 },
{ 2603.7771, 95.2933, 43 },
{ 2603.7771, 95.496, 86 },
{ 2603.7771, 95.7016, 129 },
{ 2603.7771, 95.8743, 172 },
{ 2623.0913, 92.8707, 0 },
{ 2623.0913, 93.0172, 43 },
{ 2623.0913, 93.2078, 86 },
{ 2623.0913, 93.401, 129 },
{ 2623.0913, 93.5633, 172 },
{ 2642.4053, 90.604, 0 },
{ 2642.4053, 90.7412, 43 },
{ 2642.4053, 90.9196, 86 },
{ 2642.4053, 91.1005, 129 },
{ 2642.4053, 91.2524, 172 },
{ 2661.7192, 88.3373, 0 },
{ 2661.7192, 88.4652, 43 },
{ 2661.7192, 88.6315, 86 },
{ 2661.7192, 88.8, 129 },
{ 2661.7192, 88.9415, 172 },
{ 2681.0332, 86.0706, 0 },
{ 2681.0332, 86.1892, 43 },
{ 2681.0332, 86.3433, 86 },
{ 2681.0332, 86.4994, 129 },
{ 2681.0332, 86.6305, 172 },
{ 2700.3472, 83.8039, 0 },
{ 2700.3472, 83.9132, 43 },
{ 2700.3472, 84.0551, 86 },
{ 2700.3472, 84.1989, 129 },
{ 2700.3472, 84.3196, 172 },
{ 2719.6614, 81.5371, 0 },
{ 2719.6614, 81.6371, 43 },
{ 2719.6614, 81.7669, 86 },
{ 2719.6614, 81.8983, 129 },
{ 2719.6614, 82.0087, 172 },
{ 2738.9756, 79.0085, 0 },
{ 2738.9756, 79.0932, 43 },
{ 2738.9756, 79.2032, 86 },
{ 2738.9756, 79.3146, 129 },
{ 2738.9756, 79.4082, 172 },
{ 2758.2896, 76.1567, 0 },
{ 2758.2896, 76.2346, 43 },
{ 2758.2896, 76.3358, 86 },
{ 2758.2896, 76.4383, 129 },
{ 2758.2896, 76.5243, 172 },
{ 2777.6035, 73.3048, 0 },
{ 2777.6035, 73.376, 43 },
{ 2777.6035, 73.4683, 86 },
{ 2777.6035, 73.5619, 129 },
{ 2777.6035, 73.6405, 172 },
{ 2796.9175, 70.453, 0 },
{ 2796.9175, 70.5173, 43 },
{ 2796.9175, 70.6009, 86 },
{ 2796.9175, 70.6855, 129 },
{ 2796.9175, 70.7566, 172 },
{ 2816.2314, 67.6011, 0 },
{ 2816.2314, 67.6587, 43 },
{ 2816.2314, 67.7335, 86 },
{ 2816.2314, 67.8091, 129 },
{ 2816.2314, 67.8727, 172 },
{ 2835.5454, 64.7493, 0 },
{ 2835.5454, 64.8001, 43 },
{ 2835.5454, 64.866, 86 },
{ 2835.5454, 64.9328, 129 },
{ 2835.5454, 64.9888, 172 },
{ 2854.8596, 61.8974, 0 },
{ 2854.8596, 61.9414, 43 },
{ 2854.8596, 61.9986, 86 },
{ 2854.8596, 62.0564, 129 },
{ 2854.8596, 62.1049, 172 },
{ 2874.1738, 58.8227, 0 },
{ 2874.1738, 58.8564, 43 },
{ 2874.1738, 58.9001, 86 },
{ 2874.1738, 58.9443, 129 },
{ 2874.1738, 58.9815, 172 },
{ 2893.4878, 54.8936, 0 },
{ 2893.4878, 54.9227, 43 },
{ 2893.4878, 54.9604, 86 },
{ 2893.4878, 54.9986, 129 },
{ 2893.4878, 55.0307, 172 },
{ 2912.8018, 50.9644, 0 },
{ 2912.8018, 50.9889, 43 },
{ 2912.8018, 51.0207, 86 },
{ 2912.8018, 51.053, 129 },
{ 2912.8018, 51.08, 172 },
{ 2932.1157, 47.0352, 0 },
{ 2932.1157, 47.0552, 43 },
{ 2932.1157, 47.0811, 86 },
{ 2932.1157, 47.1073, 129 },
{ 2932.1157, 47.1293, 172 },
{ 2951.4297, 43.106, 0 },
{ 2951.4297, 43.1214, 43 },
{ 2951.4297, 43.1414, 86 },
{ 2951.4297, 43.1616, 129 },
{ 2951.4297, 43.1785, 172 },
{ 2970.7439, 38.9791, 0 },
{ 2970.7439, 38.9881, 43 },
{ 2970.7439, 38.9999, 86 },
{ 2970.7439, 39.0118, 129 },
{ 2970.7439, 39.0217, 172 },
{ 2990.0579, 32.5008, 0 },
{ 2990.0579, 32.5073, 43 },
{ 2990.0579, 32.5157, 86 },
{ 2990.0579, 32.5242, 129 },
{ 2990.0579, 32.5313, 172 },
{ 3009.3721, 26.0224, 0 },
{ 3009.3721, 26.0263, 43 },
{ 3009.3721, 26.0314, 86 },
{ 3009.3721, 26.0365, 129 },
{ 3009.3721, 26.0408, 172 },
{ 3028.686, 19.4711, 0 },
{ 3028.686, 19.4723, 43 },
{ 3028.686, 19.4738, 86 },
{ 3028.686, 19.4753, 129 },
{ 3028.686, 19.4766, 172 },
{ 3048, 0, 0 },
{ 3048, 0, 43 },
{ 3048, 0, 86 },
{ 3048, 0, 129 },
{ 3048, 0, 172 },
};
#endif
