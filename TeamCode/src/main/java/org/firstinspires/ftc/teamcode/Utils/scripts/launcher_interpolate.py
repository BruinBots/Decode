import requests

BASE = 'http://192.168.43.1:8082/download?path=' # 'http://www.example.com/'
PATH = '' # TODO: Set PATH

resp = requests.get(BASE+PATH, timeout=3)
if resp.status_code == 200:
    print(resp.text)