

def rems_update():
    import sys
    import subprocess

    # implement pip as a subprocess:
    subprocess.check_call(
        [sys.executable, '-m', 'pip', 'install', '-q', 'git+https://github.com/Suke0811/REMS.git@main'])

