![ShanghaiCenturalAvenueRightTurn-ChasePlot](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/f065cddf-2a65-4574-99b6-46569a9942b5)# Decision-making in Autonomous Driving using Monte Carlo Tree Search (MCTS)

This repository contains the code, datasets, and supplementary materials related to our research paper on leveraging MCTS for decision-making in autonomous vehicles. 

## Abstract

We present a comprehensive framework based on Monte Carlo Tree Search for decision-making in autonomous driving scenarios. Through extensive simulations in MATLAB's autonomous driving toolbox 2023a (Note that some functions may not supported for lower version.). We showcase the framework's efficacy across various driving conditions, from intricate urban intersections to highway exits. While our simulations demonstrate promising results, we highlight areas for potential improvement and suggest future research directions.

## Setup & Usage

1. Clone the repository
2. Choose or set the environment. Make sure the version of MATLAB is above 2023a and the version of Automated Driving Toolbox is above 3.7.
3. Run mctsPlanning.m

## Qualitative Results


##  Shanghai Real Road Scenarios
We examine representative urban environments, such as crossing intersections, handling cut-ins, and exiting highways. The MCTS framework provides human-like, efficient decision-making across these scenarios. 


## Road in Shanghai
<table>
  <tr>
    <td><img src="https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/83dd9359-907e-4bb6-aa78-44418a43dc49" alt="ShanghaiCenturalAvenue" width="100%"/></td>
    <td><img src="https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/ce8fe49c-74ca-403a-9819-9605335e1412" alt="ShanghaiIntersection" width="100%"/></td>
    <td><img src="https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/4b419c0f-5f5b-4bbd-83de-c18befa8b5d1" alt="ShanghaiLujiazuiTunnel" width="100%"/></td>
    <td><img src="https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/4b71eab5-08ce-4cfa-9587-a5bca752ba23" alt="ShanghaiLujiazuiTunnel" width="100%"/></td>
</table>


<!--
<img src="https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/a7841a81-12d8-469b-b5a1-52c089a50d64" width="300" alt="Description of GIF">
_Figure 1: Shanghai Century Avenue._
-->

<!--![ShanghaiCenturalAvenueGoStraight-BirdEye](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/e8ac453f-2995-4149-b5e7-26a657f290ee)-->
<!--![ShanghaiCenturalAvenueGoStraight-ChasePlot](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/a7841a81-12d8-469b-b5a1-52c089a50d64)-->

### Shanghai Results 1
<table>
  <tr>
    <td><img src="https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/f39fd4e6-8a88-42dc-a896-6dad7084765c" alt="ShanghaiCenturalAvenue" width="100%"/></td>
    <td><img src="https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/05b413b6-b42b-4686-9ebc-001ba0a88ad9" alt="ShanghaiIntersection" width="100%"/></td>
</table>


### Shanghai Results 2
![ShanghaiCenturalAvenueRightTurn-BirdEye](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/8524e6b3-dd5b-430e-94a0-4d13e3839cea)
![ShanghaiCenturalAvenueRightTurn-ChasePlot](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/4fcd2284-5c00-403a-b3ec-b0dd48a87711)

### Shanghai Results 3
![ShanghaiLujiazuiTunnel-BirdEye](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/71c72961-972f-42e4-ad4a-779e679ae5a6)


### Shanghai Results 4
![ShanghaiRoundAbout-BirdEye](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/581688f7-3c00-4485-86b7-b4c877c57e54)



##  Typical Highway Scenarios
 
### Qualitative Results 1
![ds4_lanes_highWay_advanced](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/cb822561-bc23-4ace-8c89-3525fa4b9c68)
_Figure 1: Highway Exit (HE) example._

 
### Qualitative Results 2
![large_curvature](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/b05f2430-7404-40a0-ae2e-2f172bc2ddd5)
_Figure 2: Large Curvature example._

##  Typical Urban Scenarios

 
### Qualitative Results 3
![ds6_lanes_roadWith5Cars_2Cars_CuttingIn](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/ed08e5d3-db53-410c-9f08-d6a2746dac0f)
#### _Figure 3: Intersection, Straight example._
 
### Qualitative Results 4
![ds6_lanes_roadWith5Cars_egoTurningLeft](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/18a29055-820c-48cc-8276-66854dd5b6a3)
#### _Figure 4: Intersection, Unprotected Left Turn example._

 
### Qualitative Results 5
![ds6_lanes_roadWith5Cars_egoTurningLeft_2](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/6061074a-23c6-4cf0-9f1c-1d8580980a2d)
#### _Figure 5: Intersection, Unprotected Left Turn example 2._

 
### Qualitative Results 6
![ds6_lanes_roadWith5Cars_egoTurningRight_2](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/999695b9-90c9-4db8-92bb-0239679cbd35)
#### _Figure 6: Intersection, Unprotected Right Turn example._
 
### Qualitative Results 7
![ds6_lanes_roadWith5Cars_horizontal_crossing](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/23ce25f6-fecd-4872-91a0-a05a0851cb28)
#### _Figure 7: Intersection, Unprotected Straight Cross example._
 
### Qualitative Results 8
![ds6_lanes_roadWith5Cars_stopping](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/2905e5ab-d3da-4bff-afed-3ca04cfdf802)
#### _Figure 8: Intersection, Blocked example._
 
### Qualitative Results 9
![ds6_lanes_roadWith5Cars_stucked](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/38c22f47-42e7-4cc5-9a52-9dad6c68906c)
#### _Figure 9: Intersection, Blocked by Stationary Objects example._
 
### Qualitative Results 10
![ds6_lanes_roadWith5CarsCuttingIn](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/dc96ed33-df67-476e-9ac4-8d6b1c2caabc)
#### _Figure 10: Intersection, Go Straight example._

 
### Qualitative Results 11
![ds6_lanes_roadWith5CarsTurningLeft](https://github.com/zhongshun/MCTS_for_Behavior_Planning/assets/14044932/528976bf-5494-4f5f-929a-8a0214f10c01)
#### _Figure 11: Intersection, Go Straight example 2._



