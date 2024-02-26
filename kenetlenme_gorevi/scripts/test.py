from ultralytics.utils.benchmarks import benchmark

# Benchmark on GPU
benchmark(model='/home/valvarn/Savasan2024/kenetlenme_gorevi/models/best_n.pt', 
            data='/home/valvarn/Savasan2024/kenetlenme_gorevi/cfg/data.yaml', imgsz=640, half=False, device=0)