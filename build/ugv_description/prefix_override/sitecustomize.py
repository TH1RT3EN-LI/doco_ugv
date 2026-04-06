import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/th1rt3en/doco/dev_full/src/workspace/doco_ugv/install/ugv_description'
