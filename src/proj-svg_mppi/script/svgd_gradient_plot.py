#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

csv_path = "/tmp/svgd_gradient_log.csv"

df = pd.read_csv(csv_path)
# => DataFrame columns: Iteration, SampleIdx, Row, Col, GradValue

# (1) 우선, 어떤 Row를 처리할지 고민
#    - Row별로 다 그리면 너무 복잡할 수 있으니,
#      Row를 전부 평균 or 특정 Row만 관찰하는 등 다양한 방법이 가능.

# 여기서는 "Row dimension"도 모두 평균하여 "각 dimension(Col)별"만 iteration에 따라 보는 예시를 보여줍니다.
# => "Iteration, SampleIdx, Col" 별로 GradValue를 평균해서 그리자.

grouped = df.groupby(["Iteration","SampleIdx","Col"], as_index=False)["GradValue"].mean()
# => 이로써 row별 값은 평균 -> (Iteration, SampleIdx, Col)당 하나의 평균 값

# 가이드 파티클( SampleIdx )들이 몇 개 있는지 확인
unique_particles = grouped["SampleIdx"].unique()
num_particles = len(unique_particles)

fig, axes = plt.subplots(num_particles, 1, figsize=(8, 4*num_particles), sharex=True)
if num_particles == 1:
    axes = [axes]

for ax, sample_idx in zip(axes, unique_particles):
    # 이 가이드 파티클에 해당하는 데이터만 뽑기
    sub_df = grouped[grouped["SampleIdx"] == sample_idx]
    # pivot: index=Iteration, columns=Col, values=GradValue
    pivot_df = sub_df.pivot(index="Iteration", columns="Col", values="GradValue")

    # pivot_df의 col=0,1,2 => Vx, Vy, Wz라고 가정
    # iteration이 index. plot하면 dimension별 라인을 그림
    pivot_df.plot(ax=ax, marker='o')

    # 그래프 설정
    ax.set_title(f"Guide Particle #{sample_idx} - (Averaged over all Rows)")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Mean Gradient Value")
    ax.grid(True)

plt.tight_layout()
plt.savefig("svgd_gradient_eachparticle.png")
plt.show()
