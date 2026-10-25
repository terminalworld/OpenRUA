---
summary: Benchmark results and model comparisons reported in the OpenRUA paper
read_when:
  - You want the evaluation scope behind the headline results
  - You want to compare the paper's agent and model configurations
---

# Paper results

Results below are from our [paper](https://arxiv.org/abs/2610.02459), evaluated
zero-shot in simulation without privileged simulator state or access to the
benchmark's success predicate during execution.

Main benchmark results use Claude Code with Claude Opus 5 at high reasoning effort:

| Benchmark | Evaluation scope | Success |
|---|---|---|
| CaP-Bench | Seven manipulation tasks | 99.0% |
| LIBERO-PRO | All four suites, both perturbation types | 87.0% |

The model comparison uses the **LIBERO-10 suite of LIBERO-PRO**, with both
perturbation types and 200 trials per configuration:

| Agent | Model (reasoning effort) | Success |
|---|---|---|
| Claude Code | Claude Sonnet 5 (high) | 24.0% |
| Claude Code | Claude Opus 5 (high) | 72.5% |
| Claude Code | Claude Fable 5.1 (high) | 75.0% |
| Codex | GPT-6 Astra (medium) | 62.5% |
| Codex | GPT-6 Astra (high) | 69.0% |

See the paper for baselines, evaluation protocols and detailed analysis.
