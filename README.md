### CÁCH LẤY VIDEO XE TRÊN MÔ HÌNH
Để Lấy Video Xe Chạy Trên Mô Hình Về Xử Lý Vui Lòng Kết Nối Ssh Với Xe:
```ssh lhu@ip_ras``` sau đó nhập password : 123456

Tiếp tục chạy file video.py bằng lệnh:
```python3 video.py``` để kết thúc quay: ctrl + c.


Và tải video về bằng lệnh trên máy tính:
```scp lhu@ip_ras:/home/lhu/output.mp4``` sau đó nhập pass: 123456
### AUTO CAR
File main.py trong Thư mục Brain là file run tất cả các modun gồm detect lane, detect biển báo,stream video ...

Tắt các chức năng không cần thiết tại file main.py

#### Lưu Ý
Lưu ý tạo nhánh riêng trước khi commit lên git. Tránh commit nhánh master. Ai commit nhánh master sẽ bị phạt!
