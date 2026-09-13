# fable51-libero_10_task

- benchmark: libero_pro
- config: `libero_pro-fable.yaml` (sha256 55a64aa6befc)
- agent: claude-code 2.1.251, model claude-fable-5-1, operator agent
- code: openrua 0.0.6 @ 7a9e080ec25c (dirty); simulator @ 53e9966d7a8e
- images: robot 5b54a1c95d76079bf75, sandbox ac69cd743bea6b31b9e, proxy d936945ea983d870d9d
- trials: 100 recorded; 70/100 succeeded; 0 anomalies

| suite | task | seed | success | termination | wall s | turns |
|---|---|---|---|---|---|---|
| libero_10_task | 0 | 0 | yes | self_finished | 958.7 | 59 |
| libero_10_task | 0 | 1 | yes | self_finished | 817.5 | 43 |
| libero_10_task | 0 | 2 | yes | self_finished | 1041.4 | 54 |
| libero_10_task | 0 | 3 | yes | self_finished | 1481.8 | 82 |
| libero_10_task | 0 | 4 | yes | self_finished | 1093.2 | 60 |
| libero_10_task | 0 | 5 | yes | self_finished | 1424.3 | 69 |
| libero_10_task | 0 | 6 | yes | self_finished | 1105.6 | 61 |
| libero_10_task | 0 | 7 | yes | self_finished | 1192.8 | 67 |
| libero_10_task | 0 | 8 | yes | self_finished | 1699.7 | 76 |
| libero_10_task | 0 | 9 | yes | self_finished | 794.1 | 37 |
| libero_10_task | 1 | 0 | yes | self_finished | 804.2 | 54 |
| libero_10_task | 1 | 1 | yes | self_finished | 948.7 | 70 |
| libero_10_task | 1 | 2 | yes | self_finished | 870.0 | 53 |
| libero_10_task | 1 | 3 | no | self_finished | 1130.1 | 55 |
| libero_10_task | 1 | 4 | yes | self_finished | 1697.9 | 78 |
| libero_10_task | 1 | 5 | yes | self_finished | 930.5 | 56 |
| libero_10_task | 1 | 6 | yes | self_finished | 1211.8 | 68 |
| libero_10_task | 1 | 7 | no | self_finished | 918.8 | 56 |
| libero_10_task | 1 | 8 | yes | self_finished | 1679.1 | 79 |
| libero_10_task | 1 | 9 | yes | self_finished | 730.1 | 45 |
| libero_10_task | 2 | 0 | yes | self_finished | 1765.4 | 74 |
| libero_10_task | 2 | 1 | yes | self_finished | 1198.5 | 64 |
| libero_10_task | 2 | 2 | yes | self_finished | 1814.7 | 83 |
| libero_10_task | 2 | 3 | yes | self_finished | 1060.0 | 55 |
| libero_10_task | 2 | 4 | yes | self_finished | 3484.4 | 110 |
| libero_10_task | 2 | 5 | yes | self_finished | 2441.2 | 97 |
| libero_10_task | 2 | 6 | yes | self_finished | 1532.3 | 76 |
| libero_10_task | 2 | 7 | yes | self_finished | 1676.2 | 90 |
| libero_10_task | 2 | 8 | yes | self_finished | 2381.1 | 91 |
| libero_10_task | 2 | 9 | yes | self_finished | 1387.3 | 68 |
| libero_10_task | 3 | 0 | yes | self_finished | 3847.4 | 87 |
| libero_10_task | 3 | 1 | no | self_finished | 3092.0 | 97 |
| libero_10_task | 3 | 2 | no | self_finished | 3618.7 | 97 |
| libero_10_task | 3 | 3 | no | self_finished | 6639.7 | 132 |
| libero_10_task | 3 | 4 | yes | self_finished | 3371.0 | 131 |
| libero_10_task | 3 | 5 | no | self_finished | 4542.2 | 159 |
| libero_10_task | 3 | 6 | yes | self_finished | 4586.9 | 140 |
| libero_10_task | 3 | 7 | no | self_finished | 2883.9 | 88 |
| libero_10_task | 3 | 8 | yes | self_finished | 8683.4 | 211 |
| libero_10_task | 3 | 9 | yes | self_finished | 4907.3 | 138 |
| libero_10_task | 4 | 0 | yes | self_finished | 1065.2 | 56 |
| libero_10_task | 4 | 1 | yes | self_finished | 1299.9 | 69 |
| libero_10_task | 4 | 2 | yes | self_finished | 986.4 | 52 |
| libero_10_task | 4 | 3 | yes | self_finished | 906.6 | 44 |
| libero_10_task | 4 | 4 | yes | self_finished | 1824.1 | 80 |
| libero_10_task | 4 | 5 | yes | self_finished | 1081.3 | 54 |
| libero_10_task | 4 | 6 | yes | self_finished | 1299.4 | 66 |
| libero_10_task | 4 | 7 | yes | self_finished | 1126.7 | 63 |
| libero_10_task | 4 | 8 | yes | self_finished | 1508.0 | 69 |
| libero_10_task | 4 | 9 | no | self_finished | 942.9 | 48 |
| libero_10_task | 5 | 0 | yes | wall_clock_cap | 14468.0 |  |
| libero_10_task | 5 | 1 | yes | self_finished | 1623.4 | 77 |
| libero_10_task | 5 | 2 | yes | self_finished | 4257.6 | 160 |
| libero_10_task | 5 | 3 | no | self_finished | 1616.8 | 73 |
| libero_10_task | 5 | 4 | yes | self_finished | 2853.7 | 108 |
| libero_10_task | 5 | 5 | yes | self_finished | 5419.1 | 138 |
| libero_10_task | 5 | 6 | no | self_finished | 10146.9 | 206 |
| libero_10_task | 5 | 7 | yes | self_finished | 11285.6 | 304 |
| libero_10_task | 5 | 8 | no | self_finished | 4461.6 | 130 |
| libero_10_task | 5 | 9 | yes | self_finished | 2170.6 | 95 |
| libero_10_task | 6 | 0 | no | self_finished | 2067.7 | 67 |
| libero_10_task | 6 | 1 | yes | self_finished | 1095.9 | 62 |
| libero_10_task | 6 | 2 | yes | self_finished | 1146.0 | 61 |
| libero_10_task | 6 | 3 | yes | self_finished | 4359.0 | 132 |
| libero_10_task | 6 | 4 | no | self_finished | 1156.5 | 53 |
| libero_10_task | 6 | 5 | yes | self_finished | 1132.5 | 62 |
| libero_10_task | 6 | 6 | no | self_finished | 1263.1 | 65 |
| libero_10_task | 6 | 7 | yes | self_finished | 1170.1 | 62 |
| libero_10_task | 6 | 8 | yes | self_finished | 1382.2 | 64 |
| libero_10_task | 6 | 9 | yes | self_finished | 1046.3 | 48 |
| libero_10_task | 7 | 0 | no | self_finished | 1542.5 | 69 |
| libero_10_task | 7 | 1 | yes | self_finished | 1645.8 | 76 |
| libero_10_task | 7 | 2 | yes | self_finished | 1283.2 | 68 |
| libero_10_task | 7 | 3 | no | self_finished | 1408.4 | 82 |
| libero_10_task | 7 | 4 | yes | self_finished | 1519.6 | 70 |
| libero_10_task | 7 | 5 | yes | self_finished | 1454.0 | 82 |
| libero_10_task | 7 | 6 | no | self_finished | 1693.3 | 95 |
| libero_10_task | 7 | 7 | yes | self_finished | 1883.0 | 73 |
| libero_10_task | 7 | 8 | yes | self_finished | 1002.0 | 51 |
| libero_10_task | 7 | 9 | yes | self_finished | 1358.0 | 89 |
| libero_10_task | 8 | 0 | no | self_finished | 1579.9 | 64 |
| libero_10_task | 8 | 1 | no | self_finished | 6424.2 | 141 |
| libero_10_task | 8 | 2 | no | self_finished | 2443.6 | 82 |
| libero_10_task | 8 | 3 | no | self_finished | 1710.3 | 64 |
| libero_10_task | 8 | 4 | no | self_finished | 1772.5 | 69 |
| libero_10_task | 8 | 5 | no | self_finished | 1306.7 | 50 |
| libero_10_task | 8 | 6 | no | self_finished | 1381.8 | 57 |
| libero_10_task | 8 | 7 | no | self_finished | 1210.8 | 68 |
| libero_10_task | 8 | 8 | no | self_finished | 1838.2 | 64 |
| libero_10_task | 8 | 9 | no | self_finished | 4672.3 | 114 |
| libero_10_task | 9 | 0 | yes | self_finished | 6306.6 | 205 |
| libero_10_task | 9 | 1 | yes | self_finished | 5198.6 | 131 |
| libero_10_task | 9 | 2 | yes | self_finished | 6056.1 | 147 |
| libero_10_task | 9 | 3 | no | self_finished | 8467.2 | 220 |
| libero_10_task | 9 | 4 | yes | self_finished | 7340.5 | 173 |
| libero_10_task | 9 | 5 | yes | self_finished | 11137.6 | 304 |
| libero_10_task | 9 | 6 | yes | self_finished | 7453.2 | 233 |
| libero_10_task | 9 | 7 | yes | self_finished | 4453.1 | 150 |
| libero_10_task | 9 | 8 | no | wall_clock_cap | 14448.5 |  |
| libero_10_task | 9 | 9 | no | self_finished | 7574.5 | 163 |
