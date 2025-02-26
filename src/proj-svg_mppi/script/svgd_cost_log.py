#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

# CSV 파일 경로
csv_path = "/tmp/svgd_cost_log.csv"

# CSV 읽기
df = pd.read_csv(csv_path)
# => columns: ["Iteration", "SampleIdx", "Cost"]

# (A) 1) Iteration별 SampleIdx별 Cost를 라인 플롯으로 모두 보고 싶다면:
# pivot을 써서 (Iteration을 index, SampleIdx를 columns) -> Cost를 value
pivot_df = df.pivot(index="Iteration", columns="SampleIdx", values="Cost")

plt.figure(figsize=(8,6))
# pivot_df의 각 column(=SampleIdx)에 대해 라인을 그림
pivot_df.plot(marker='o')
plt.title("Cost for Each Guide Sample over Iteration")
plt.xlabel("Iteration")
plt.ylabel("Cost")
plt.grid(True)
plt.legend(title="SampleIdx")
plt.savefig("svgd_cost_all_samples.png")
plt.show()

# (B) 2) Iteration별로 'min cost'를 보고 싶다면
min_costs = df.groupby("Iteration")["Cost"].min()
plt.figure(figsize=(8,5))
plt.plot(min_costs.index, min_costs.values, marker='o', color='red')
plt.title("SVGD Min Cost per Iteration")
plt.xlabel("Iteration")
plt.ylabel("Min Cost among samples")
plt.grid(True)
plt.savefig("svgd_min_cost.png")
plt.show()

# (C) 3) Iteration별로 'mean cost'도
mean_costs = df.groupby("Iteration")["Cost"].mean()
plt.figure(figsize=(8,5))
plt.plot(mean_costs.index, mean_costs.values, marker='x', color='blue')
plt.title("SVGD Mean Cost per Iteration")
plt.xlabel("Iteration")
plt.ylabel("Mean Cost among samples")
plt.grid(True)
plt.savefig("svgd_mean_cost.png")
plt.show()
