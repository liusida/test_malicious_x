# Usage:

Create Conda Envrionment
```bash
conda create -y -n test_malicious_x python=3.8.5
source activate test_malicious_x
```

Install Requirements
```bash
pip install -r requirements.txt
```

Run Control Experiments
```bash
python test_malicious_x.py --seed=0
```

Run Treatment Experiments
```bash
python test_malicious_x.py --seed=0 -x
```

View Results
```bash
tensorboard --logdir=tb
```

# Results:

## without "malicious" x

![Without X](images/v0.png)

## with "malicious" x

![With X](images/v1.png)
