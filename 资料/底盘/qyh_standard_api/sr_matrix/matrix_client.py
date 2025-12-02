"""
Standard Robots Matrix HTTP API Client
通过 HTTP API 与 Matrix 系统通信，实现登录、获取地图等功能
"""

import requests
import json
import time
import hashlib
from typing import Optional, Dict, Any, List
from urllib.parse import urljoin


class MatrixClient:
    """Matrix 系统 API 客户端"""
    
    def __init__(self, host: str, port: int = 80, timeout: int = 10):
        """
        初始化 Matrix 客户端
        
        Args:
            host: Matrix 系统 IP 地址
            port: HTTP 端口 (默认 80)
            timeout: 请求超时时间 (秒)
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.base_url = f"http://{host}:{port}" if port != 80 else f"http://{host}"
        
        # Session 用于保持登录状态
        self.session = requests.Session()
        self.session.headers.update({
            "Content-Type": "application/json",
            "Accept": "application/json, text/plain, */*",
            "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36"
        })
        
        # 登录状态
        self.logged_in = False
        self.token: Optional[str] = None
        self.user_info: Optional[Dict] = None
    
    def _url(self, path: str) -> str:
        """构建完整 URL"""
        return urljoin(self.base_url, path)
    
    def _request(self, method: str, path: str, **kwargs) -> requests.Response:
        """
        发送 HTTP 请求
        
        Args:
            method: HTTP 方法 (GET, POST, etc.)
            path: API 路径
            **kwargs: 传递给 requests 的参数
            
        Returns:
            Response 对象
        """
        url = self._url(path)
        kwargs.setdefault("timeout", self.timeout)
        
        # 如果有 token，添加到 header
        if self.token:
            self.session.headers["Authorization"] = f"Bearer {self.token}"
        
        response = self.session.request(method, url, **kwargs)
        return response
    
    def _get(self, path: str, **kwargs) -> requests.Response:
        """GET 请求"""
        return self._request("GET", path, **kwargs)
    
    def _post(self, path: str, data: Any = None, **kwargs) -> requests.Response:
        """POST 请求"""
        if data is not None:
            kwargs["json"] = data
        return self._request("POST", path, **kwargs)
    
    def check_connection(self) -> bool:
        """
        检查与 Matrix 系统的连接
        
        Returns:
            是否可以连接
        """
        try:
            response = self._get("/", timeout=5)
            return response.status_code == 200
        except Exception as e:
            print(f"连接检查失败: {e}")
            return False
    
    def login(self, username: str, password: str) -> bool:
        """
        登录 Matrix 系统
        
        Args:
            username: 用户名
            password: 密码
            
        Returns:
            是否登录成功
        """
        print(f"正在登录 Matrix 系统: {self.base_url}")
        print(f"用户名: {username}")
        
        # Standard Robots Matrix 可能使用的登录 API 端点
        login_endpoints = [
            "/api/user/login",
            "/api/auth/login", 
            "/api/login",
            "/user/login",
            "/auth/login",
            "/login",
            "/api/v1/user/login",
            "/api/v1/auth/login",
        ]
        
        # 尝试不同的登录数据格式
        login_payloads = [
            # 标准格式
            {"username": username, "password": password},
            # 带类型
            {"username": username, "password": password, "type": "account"},
            # 用户名/密码字段变体
            {"userName": username, "passWord": password},
            {"user": username, "pass": password},
            {"account": username, "password": password},
            # MD5 密码
            {"username": username, "password": hashlib.md5(password.encode()).hexdigest()},
        ]
        
        for endpoint in login_endpoints:
            for payload in login_payloads:
                try:
                    print(f"  尝试: {endpoint} with {list(payload.keys())}")
                    response = self._post(endpoint, data=payload)
                    
                    if response.status_code == 200:
                        try:
                            result = response.json()
                            print(f"    响应: {result}")
                            
                            # 检查是否登录成功
                            if self._check_login_success(result):
                                self._extract_token(result)
                                self.logged_in = True
                                print(f"✅ 登录成功!")
                                return True
                        except json.JSONDecodeError:
                            # 可能返回非 JSON
                            if "success" in response.text.lower():
                                self.logged_in = True
                                return True
                    elif response.status_code == 401:
                        print(f"    认证失败 (401)")
                    elif response.status_code == 404:
                        continue  # 端点不存在，尝试下一个
                    else:
                        print(f"    状态码: {response.status_code}")
                        
                except requests.exceptions.Timeout:
                    print(f"    超时")
                except requests.exceptions.ConnectionError:
                    print(f"    连接失败")
                except Exception as e:
                    print(f"    错误: {e}")
        
        print("❌ 所有登录尝试均失败")
        return False
    
    def _check_login_success(self, result: Dict) -> bool:
        """检查登录响应是否表示成功"""
        # 检查常见的成功标志
        if isinstance(result, dict):
            # code == 0 或 200 通常表示成功
            code = result.get("code", result.get("status", result.get("errcode")))
            if code in [0, 200, "0", "200"]:
                return True
            
            # success 字段
            if result.get("success") == True:
                return True
            
            # 有 token 或 data 通常表示成功
            if result.get("token") or result.get("data", {}).get("token"):
                return True
            if result.get("accessToken") or result.get("access_token"):
                return True
        
        return False
    
    def _extract_token(self, result: Dict):
        """从登录响应中提取 token"""
        if isinstance(result, dict):
            # 直接在顶层
            self.token = result.get("token") or result.get("accessToken") or result.get("access_token")
            
            # 在 data 中
            if not self.token and "data" in result:
                data = result["data"]
                if isinstance(data, dict):
                    self.token = data.get("token") or data.get("accessToken") or data.get("access_token")
                    self.user_info = data.get("user") or data.get("userInfo")
            
            if self.token:
                print(f"  Token: {self.token[:20]}...")
    
    def logout(self) -> bool:
        """登出"""
        if not self.logged_in:
            return True
        
        try:
            # 尝试登出
            logout_endpoints = ["/api/user/logout", "/api/logout", "/logout"]
            for endpoint in logout_endpoints:
                try:
                    self._post(endpoint)
                except:
                    pass
        finally:
            self.logged_in = False
            self.token = None
            self.session.cookies.clear()
        
        return True
    
    # ==================== 地图相关 API ====================
    
    def get_map_list(self) -> List[Dict]:
        """
        获取地图列表
        
        Returns:
            地图信息列表
        """
        if not self.logged_in:
            print("请先登录")
            return []
        
        map_endpoints = [
            "/api/map/list",
            "/api/maps",
            "/api/v1/map/list",
            "/map/list",
            "/api/map/getList",
            "/api/navigation/maps",
        ]
        
        for endpoint in map_endpoints:
            try:
                response = self._get(endpoint)
                if response.status_code == 200:
                    result = response.json()
                    print(f"地图列表响应: {result}")
                    
                    # 提取地图列表
                    maps = self._extract_list(result, "maps", "mapList", "data", "list")
                    if maps:
                        return maps
            except Exception as e:
                continue
        
        return []
    
    def get_current_map(self) -> Optional[Dict]:
        """
        获取当前使用的地图信息
        
        Returns:
            当前地图信息
        """
        if not self.logged_in:
            print("请先登录")
            return None
        
        endpoints = [
            "/api/map/current",
            "/api/map/getCurrent",
            "/api/v1/map/current",
            "/api/navigation/currentMap",
            "/api/robot/currentMap",
        ]
        
        for endpoint in endpoints:
            try:
                response = self._get(endpoint)
                if response.status_code == 200:
                    result = response.json()
                    print(f"当前地图响应: {result}")
                    return result.get("data", result)
            except:
                continue
        
        return None
    
    def get_map_data(self, map_name: str = None, map_id: str = None) -> Optional[Dict]:
        """
        获取地图数据 (包含图片和元信息)
        
        Args:
            map_name: 地图名称
            map_id: 地图 ID
            
        Returns:
            地图数据
        """
        if not self.logged_in:
            print("请先登录")
            return None
        
        # 构建请求参数
        params = {}
        if map_name:
            params["name"] = map_name
            params["mapName"] = map_name
        if map_id:
            params["id"] = map_id
            params["mapId"] = map_id
        
        endpoints = [
            "/api/map/data",
            "/api/map/get",
            "/api/map/detail",
            f"/api/map/{map_id}" if map_id else None,
            f"/api/map/{map_name}" if map_name else None,
        ]
        
        for endpoint in filter(None, endpoints):
            try:
                response = self._get(endpoint, params=params)
                if response.status_code == 200:
                    result = response.json()
                    print(f"地图数据响应: {json.dumps(result, indent=2, ensure_ascii=False)[:500]}")
                    return result.get("data", result)
            except:
                continue
        
        return None
    
    def get_map_image(self, map_name: str = None, map_id: str = None) -> Optional[bytes]:
        """
        获取地图图片
        
        Args:
            map_name: 地图名称
            map_id: 地图 ID
            
        Returns:
            图片二进制数据
        """
        if not self.logged_in:
            print("请先登录")
            return None
        
        # 构建请求参数
        params = {}
        if map_name:
            params["name"] = map_name
        if map_id:
            params["id"] = map_id
        
        endpoints = [
            "/api/map/image",
            "/api/map/png",
            "/api/map/picture",
            f"/api/map/{map_id}/image" if map_id else None,
            f"/api/map/{map_name}/image" if map_name else None,
            "/api/map/getImage",
        ]
        
        for endpoint in filter(None, endpoints):
            try:
                response = self._get(endpoint, params=params)
                if response.status_code == 200:
                    content_type = response.headers.get("Content-Type", "")
                    if "image" in content_type or len(response.content) > 1000:
                        print(f"获取到地图图片: {len(response.content)} bytes")
                        return response.content
            except:
                continue
        
        return None
    
    def get_sites(self, map_name: str = None) -> List[Dict]:
        """
        获取站点列表
        
        Args:
            map_name: 地图名称 (可选)
            
        Returns:
            站点列表
        """
        if not self.logged_in:
            print("请先登录")
            return []
        
        params = {"mapName": map_name} if map_name else {}
        
        endpoints = [
            "/api/site/list",
            "/api/sites",
            "/api/navigation/sites",
            "/api/map/sites",
            "/api/v1/site/list",
        ]
        
        for endpoint in endpoints:
            try:
                response = self._get(endpoint, params=params)
                if response.status_code == 200:
                    result = response.json()
                    sites = self._extract_list(result, "sites", "siteList", "data", "list")
                    if sites:
                        print(f"获取到 {len(sites)} 个站点")
                        return sites
            except:
                continue
        
        return []
    
    # ==================== 机器人状态 API ====================
    
    def get_robot_status(self) -> Optional[Dict]:
        """
        获取机器人状态
        
        Returns:
            机器人状态信息
        """
        if not self.logged_in:
            print("请先登录")
            return None
        
        endpoints = [
            "/api/robot/status",
            "/api/robot/state",
            "/api/status",
            "/api/v1/robot/status",
            "/api/navigation/status",
        ]
        
        for endpoint in endpoints:
            try:
                response = self._get(endpoint)
                if response.status_code == 200:
                    result = response.json()
                    return result.get("data", result)
            except:
                continue
        
        return None
    
    def get_robot_pose(self) -> Optional[Dict]:
        """
        获取机器人位姿
        
        Returns:
            {x, y, yaw} 位姿信息
        """
        if not self.logged_in:
            print("请先登录")
            return None
        
        endpoints = [
            "/api/robot/pose",
            "/api/robot/position",
            "/api/navigation/pose",
            "/api/v1/robot/pose",
        ]
        
        for endpoint in endpoints:
            try:
                response = self._get(endpoint)
                if response.status_code == 200:
                    result = response.json()
                    return result.get("data", result)
            except:
                continue
        
        return None
    
    # ==================== 辅助方法 ====================
    
    def _extract_list(self, result: Dict, *keys) -> List:
        """从响应中提取列表数据"""
        if isinstance(result, list):
            return result
        
        if isinstance(result, dict):
            for key in keys:
                if key in result:
                    val = result[key]
                    if isinstance(val, list):
                        return val
            
            # 递归检查 data 字段
            if "data" in result:
                return self._extract_list(result["data"], *keys)
        
        return []
    
    def explore_api(self) -> Dict[str, Any]:
        """
        探索 API 端点，用于发现可用的 API
        
        Returns:
            发现的 API 信息
        """
        discovered = {
            "working_endpoints": [],
            "api_info": None
        }
        
        # 常见的 API 发现端点
        discovery_endpoints = [
            "/api",
            "/api/v1",
            "/swagger",
            "/swagger.json",
            "/api-docs",
            "/openapi.json",
            "/api/info",
            "/api/version",
        ]
        
        for endpoint in discovery_endpoints:
            try:
                response = self._get(endpoint, timeout=3)
                if response.status_code == 200:
                    discovered["working_endpoints"].append({
                        "endpoint": endpoint,
                        "content_type": response.headers.get("Content-Type"),
                        "sample": response.text[:200]
                    })
            except:
                pass
        
        return discovered
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.logout()
        self.session.close()
