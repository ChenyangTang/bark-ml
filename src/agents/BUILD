py_library(
    name = "agents",
    srcs = ["base_agent.py",
            "sac_agent.py",
            "sac_agent_cgnn.py",
            "td3_agent.py",
            "ppo_agent.py",
            "tfa_agent.py"],
    visibility = ["//visibility:public"],
    deps = ["@icml2020//source:cgnn",
            "@icml2020//source:tfa_networks"],
)