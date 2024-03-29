from django.urls import path
from . import views, consumers

urlpatterns = [
    path('', views.home, name='home'),
    path("api/action/", views.action, name='action'),
    path("api/get_all_histories", views.get_all_histories, name='histories')
]


websocket_urlpatterns = [
    path("ws/camera/", consumers.ChatConsumer.as_asgi()),
]
