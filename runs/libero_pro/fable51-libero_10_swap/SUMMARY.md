# fable51-libero_10_swap

- benchmark: libero_pro
- config: `libero_pro-fable.yaml` (sha256 55a64aa6befc)
- agent: claude-code 2.1.251, model claude-fable-5-1, operator agent
- code: openrua 0.0.6 @ 7a9e080ec25c (dirty); simulator @ 53e9966d7a8e
- images: robot 5b54a1c95d76079bf75, sandbox ac69cd743bea6b31b9e, proxy d936945ea983d870d9d
- trials: 100 recorded; 80/100 succeeded; 0 anomalies

| suite | task | seed | success | termination | wall s | turns |
|---|---|---|---|---|---|---|
| libero_10_swap | 0 | 0 | yes | self_finished | 1806.7 | 76 |
| libero_10_swap | 0 | 1 | yes | self_finished | 1008.3 | 44 |
| libero_10_swap | 0 | 2 | yes | self_finished | 4808.1 | 152 |
| libero_10_swap | 0 | 3 | yes | self_finished | 4645.4 | 139 |
| libero_10_swap | 0 | 4 | yes | self_finished | 1971.0 | 87 |
| libero_10_swap | 0 | 5 | yes | self_finished | 1438.0 | 76 |
| libero_10_swap | 0 | 6 | yes | self_finished | 1770.6 | 86 |
| libero_10_swap | 0 | 7 | no | self_finished | 1082.0 | 56 |
| libero_10_swap | 0 | 8 | yes | self_finished | 1353.6 | 58 |
| libero_10_swap | 0 | 9 | yes | self_finished | 1472.7 | 87 |
| libero_10_swap | 1 | 0 | yes | self_finished | 1054.6 | 47 |
| libero_10_swap | 1 | 1 | yes | self_finished | 1080.0 | 50 |
| libero_10_swap | 1 | 2 | yes | self_finished | 842.2 | 51 |
| libero_10_swap | 1 | 3 | yes | self_finished | 1493.4 | 78 |
| libero_10_swap | 1 | 4 | yes | self_finished | 1384.1 | 52 |
| libero_10_swap | 1 | 5 | yes | self_finished | 1006.3 | 59 |
| libero_10_swap | 1 | 6 | no | self_finished | 1745.4 | 94 |
| libero_10_swap | 1 | 7 | yes | self_finished | 804.2 | 54 |
| libero_10_swap | 1 | 8 | yes | self_finished | 1227.2 | 63 |
| libero_10_swap | 1 | 9 | yes | self_finished | 959.5 | 49 |
| libero_10_swap | 2 | 0 | yes | self_finished | 4094.4 | 172 |
| libero_10_swap | 2 | 1 | yes | self_finished | 1676.3 | 70 |
| libero_10_swap | 2 | 2 | yes | self_finished | 1680.4 | 97 |
| libero_10_swap | 2 | 3 | yes | self_finished | 1708.5 | 77 |
| libero_10_swap | 2 | 4 | yes | self_finished | 3359.0 | 136 |
| libero_10_swap | 2 | 5 | yes | self_finished | 2410.3 | 96 |
| libero_10_swap | 2 | 6 | yes | self_finished | 1571.2 | 74 |
| libero_10_swap | 2 | 7 | yes | self_finished | 1599.1 | 80 |
| libero_10_swap | 2 | 8 | yes | self_finished | 2101.1 | 96 |
| libero_10_swap | 2 | 9 | yes | self_finished | 1515.3 | 78 |
| libero_10_swap | 3 | 0 | yes | self_finished | 3959.2 | 120 |
| libero_10_swap | 3 | 1 | yes | self_finished | 2982.9 | 116 |
| libero_10_swap | 3 | 2 | no | self_finished | 2791.6 | 105 |
| libero_10_swap | 3 | 3 | yes | self_finished | 2445.5 | 99 |
| libero_10_swap | 3 | 4 | no | self_finished | 1293.0 | 67 |
| libero_10_swap | 3 | 5 | no | self_finished | 3979.7 | 100 |
| libero_10_swap | 3 | 6 | no | self_finished | 2201.7 | 95 |
| libero_10_swap | 3 | 7 | yes | self_finished | 1850.3 | 96 |
| libero_10_swap | 3 | 8 | yes | self_finished | 2773.8 | 110 |
| libero_10_swap | 3 | 9 | yes | self_finished | 3028.6 | 111 |
| libero_10_swap | 4 | 0 | yes | self_finished | 1140.3 | 58 |
| libero_10_swap | 4 | 1 | yes | self_finished | 1188.6 | 73 |
| libero_10_swap | 4 | 2 | no | self_finished | 1207.3 | 55 |
| libero_10_swap | 4 | 3 | yes | self_finished | 3100.7 | 113 |
| libero_10_swap | 4 | 4 | yes | self_finished | 1224.3 | 66 |
| libero_10_swap | 4 | 5 | yes | self_finished | 938.3 | 51 |
| libero_10_swap | 4 | 6 | yes | self_finished | 1099.9 | 62 |
| libero_10_swap | 4 | 7 | yes | self_finished | 997.1 | 57 |
| libero_10_swap | 4 | 8 | yes | self_finished | 1539.2 | 74 |
| libero_10_swap | 4 | 9 | yes | self_finished | 1260.4 | 59 |
| libero_10_swap | 5 | 0 | yes | self_finished | 1254.6 | 69 |
| libero_10_swap | 5 | 1 | yes | self_finished | 1454.0 | 81 |
| libero_10_swap | 5 | 2 | yes | self_finished | 1153.6 | 58 |
| libero_10_swap | 5 | 3 | yes | self_finished | 1453.6 | 83 |
| libero_10_swap | 5 | 4 | yes | self_finished | 1402.8 | 65 |
| libero_10_swap | 5 | 5 | yes | self_finished | 1492.2 | 71 |
| libero_10_swap | 5 | 6 | yes | self_finished | 1339.8 | 69 |
| libero_10_swap | 5 | 7 | yes | self_finished | 1770.4 | 94 |
| libero_10_swap | 5 | 8 | yes | self_finished | 1364.5 | 72 |
| libero_10_swap | 5 | 9 | yes | self_finished | 1345.0 | 72 |
| libero_10_swap | 6 | 0 | no | self_finished | 1189.3 | 72 |
| libero_10_swap | 6 | 1 | yes | self_finished | 1575.3 | 75 |
| libero_10_swap | 6 | 2 | yes | self_finished | 1325.6 | 60 |
| libero_10_swap | 6 | 3 | no | self_finished | 1673.9 | 92 |
| libero_10_swap | 6 | 4 | no | self_finished | 1397.6 | 66 |
| libero_10_swap | 6 | 5 | yes | self_finished | 995.8 | 48 |
| libero_10_swap | 6 | 6 | no | self_finished | 1345.1 | 65 |
| libero_10_swap | 6 | 7 | no | self_finished | 1367.3 | 60 |
| libero_10_swap | 6 | 8 | no | self_finished | 1290.3 | 63 |
| libero_10_swap | 6 | 9 | no | self_finished | 5613.9 | 158 |
| libero_10_swap | 7 | 0 | yes | self_finished | 911.3 | 50 |
| libero_10_swap | 7 | 1 | yes | self_finished | 1126.0 | 57 |
| libero_10_swap | 7 | 2 | yes | self_finished | 1149.8 | 70 |
| libero_10_swap | 7 | 3 | yes | self_finished | 1652.8 | 79 |
| libero_10_swap | 7 | 4 | yes | self_finished | 1017.4 | 59 |
| libero_10_swap | 7 | 5 | yes | self_finished | 755.2 | 46 |
| libero_10_swap | 7 | 6 | yes | self_finished | 1387.8 | 67 |
| libero_10_swap | 7 | 7 | yes | self_finished | 5194.0 | 168 |
| libero_10_swap | 7 | 8 | yes | self_finished | 1659.4 | 79 |
| libero_10_swap | 7 | 9 | yes | self_finished | 739.0 | 46 |
| libero_10_swap | 8 | 0 | yes | wall_clock_cap | 14454.7 |  |
| libero_10_swap | 8 | 1 | yes | self_finished | 8954.9 | 234 |
| libero_10_swap | 8 | 2 | yes | self_finished | 1686.8 | 53 |
| libero_10_swap | 8 | 3 | yes | self_finished | 3918.6 | 132 |
| libero_10_swap | 8 | 4 | yes | self_finished | 3559.6 | 84 |
| libero_10_swap | 8 | 5 | yes | self_finished | 1754.2 | 72 |
| libero_10_swap | 8 | 6 | yes | self_finished | 11324.8 | 261 |
| libero_10_swap | 8 | 7 | yes | self_finished | 9254.8 | 170 |
| libero_10_swap | 8 | 8 | no | wall_clock_cap | 14447.3 |  |
| libero_10_swap | 8 | 9 | yes | self_finished | 3215.3 | 56 |
| libero_10_swap | 9 | 0 | yes | self_finished | 11968.5 | 244 |
| libero_10_swap | 9 | 1 | yes | self_finished | 5525.1 | 143 |
| libero_10_swap | 9 | 2 | yes | self_finished | 7880.0 | 155 |
| libero_10_swap | 9 | 3 | no | wall_clock_cap | 14451.2 |  |
| libero_10_swap | 9 | 4 | no | wall_clock_cap | 14447.9 |  |
| libero_10_swap | 9 | 5 | no | self_finished | 5598.4 | 117 |
| libero_10_swap | 9 | 6 | no | wall_clock_cap | 14446.7 |  |
| libero_10_swap | 9 | 7 | no | self_finished | 7192.1 | 135 |
| libero_10_swap | 9 | 8 | yes | self_finished | 4058.5 | 129 |
| libero_10_swap | 9 | 9 | yes | self_finished | 5699.2 | 147 |
