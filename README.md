# Decision-making in Autonomous Driving using Monte Carlo Tree Search (MCTS)

This repository contains the code, datasets, and supplementary materials related to our research paper on leveraging MCTS for decision-making in autonomous vehicles. 

## Abstract

We present a comprehensive framework based on Monte Carlo Tree Search for decision-making in autonomous driving scenarios. Through extensive simulations in MATLAB's autonomous driving toolbox, we showcase the framework's efficacy across various driving conditions, from intricate urban intersections to highway exits. While our simulations demonstrate promising results, we highlight areas for potential improvement and suggest future research directions.

## Performance in Typical Highway Scenarios
![ds4_lanes_highWay_advanced](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/cb822561-bc23-4ace-8c89-3525fa4b9c68)
_Figure 1: Highway Exit (HE) example._

![ds6_lanes_roadWith5Cars_2Cars_CuttingIn](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/ed08e5d3-db53-410c-9f08-d6a2746dac0f)

![ds6_lanes_roadWith5Cars_egoTurningLeft](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/18a29055-820c-48cc-8276-66854dd5b6a3)

![ds6_lanes_roadWith5Cars_egoTurningLeft_2](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/6061074a-23c6-4cf0-9f1c-1d8580980a2d)


![ds6_lanes_roadWith5Cars_egoTurningRight_2](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/999695b9-90c9-4db8-92bb-0239679cbd35)

![ds6_lanes_roadWith5Cars_horizontal_crossing](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/23ce25f6-fecd-4872-91a0-a05a0851cb28)

![ds6_lanes_roadWith5Cars_stopping](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/2905e5ab-d3da-4bff-afed-3ca04cfdf802)

![ds6_lanes_roadWith5Cars_stucked](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/38c22f47-42e7-4cc5-9a52-9dad6c68906c)

![ds6_lanes_roadWith5CarsCuttingIn](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/dc96ed33-df67-476e-9ac4-8d6b1c2caabc)

![Uploading ds6_lanes_roadWith5CarsTurningLeft.gif…]()


## Repository Contents

- `code/`: Contains all the source code used in our experiments.
- `data/`: Datasets and environmental setups used in simulations.
- `figures/`: Visualization and .png files related to our qualitative results.
- `docs/`: Supplementary documents, including detailed algorithm explanations.

## Key Results

1. **Qualitative Results:** We examine representative urban environments, such as crossing intersections, handling cut-ins, and exiting highways. The MCTS framework provides human-like, efficient decision-making across these scenarios. More details and GIF animations can be found in `figures/`.

2. **Quantitative Results:** The results of our simulations show MCTS's robustness across varied scenarios, especially with increased iteration counts. Detailed statistical analyses are provided in the main paper and supplementary documents.

![HE Example](path/to/your/figure2.png)
_Figure 2: Highway Exit (HE) scenario._

## Setup & Usage

1. Clone the repository:
