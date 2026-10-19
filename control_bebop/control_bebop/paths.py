"""
Rutas del proyecto independientes de la computadora y del usuario.

El proyecto corre en dos máquinas (laptop con ROS 2 Humble y escritorio con
ROS 2 Jazzy) con usuarios y ubicaciones distintas. Nada debe usar rutas
absolutas: todo se resuelve relativo al código fuente del paquete.

Cómo se encuentra el código fuente:
    1. Variable de entorno CONTROL_BEBOP_DIR, si existe.
    2. Si el paquete se instaló con --symlink-install, desde este archivo.
    3. Se sube desde el prefijo instalado (<ws>/install/control_bebop) hasta
       la carpeta install, y se busca en <ws>/src el package.xml con
       <name>control_bebop</name>.

Datos generados (bags, videos, trayectorias):
    <paquete>/bags, <paquete>/videos, <paquete>/trayectorias
    Se puede cambiar la raíz con BEBOP_DATA_DIR. Si no se encuentra el código
    fuente, se usa ~/.ros/control_bebop.

Este módulo no importa rclpy, así que sirve en nodos, launches y scripts.
"""

import functools
import os
import re
import sys
import warnings

PACKAGE = 'control_bebop'
_SKIP_DIRS = {'build', 'install', 'log', '.git', '__pycache__', 'bags', 'videos'}
_MAX_DEPTH = 4


def _is_package_dir(path):
    xml = os.path.join(path, 'package.xml')
    if not (os.path.isfile(xml) and os.path.isfile(os.path.join(path, 'setup.py'))):
        return False
    try:
        with open(xml, encoding='utf-8') as f:
            return re.search(r'<name>\s*%s\s*</name>' % PACKAGE, f.read()) is not None
    except OSError:
        return False


def _search_src(src_dir):
    """Busca el paquete en src_dir con profundidad limitada."""
    if not os.path.isdir(src_dir):
        return None
    base_depth = src_dir.rstrip(os.sep).count(os.sep)
    for root, dirs, _files in os.walk(src_dir):
        if _is_package_dir(root):
            return root
        if root.count(os.sep) - base_depth >= _MAX_DEPTH:
            dirs[:] = []
            continue
        dirs[:] = sorted(d for d in dirs
                         if d not in _SKIP_DIRS and not d.startswith('.'))
    return None


def _install_prefix():
    """Prefijo instalado del paquete, sin depender de ament_index (opcional)."""
    try:
        from ament_index_python.packages import get_package_prefix
        return get_package_prefix(PACKAGE)
    except Exception:
        return None


def workspace_from_prefix(prefix):
    """<ws> a partir de <ws>/install/<pkg> o <ws>/install (merge-install)."""
    if not prefix:
        return None
    path = os.path.abspath(prefix)
    while True:
        if os.path.basename(path) == 'install':
            return os.path.dirname(path)
        parent = os.path.dirname(path)
        if parent == path:
            return None
        path = parent


@functools.lru_cache(maxsize=None)
def package_source_dir():
    """Carpeta del código fuente de control_bebop, o None si no se encuentra."""
    env = os.environ.get('CONTROL_BEBOP_DIR')
    if env and _is_package_dir(os.path.abspath(os.path.expanduser(env))):
        return os.path.abspath(os.path.expanduser(env))

    # --symlink-install o ejecución directa desde src
    here = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    if _is_package_dir(here):
        return here

    ws = workspace_from_prefix(_install_prefix())
    if ws:
        found = _search_src(os.path.join(ws, 'src'))
        if found:
            return found
    return None


def workspace_dir():
    """Raíz del workspace de colcon (<ws>), o None."""
    ws = workspace_from_prefix(_install_prefix())
    if ws:
        return ws
    src = package_source_dir()
    if src:
        path = src
        while os.path.dirname(path) != path:
            if os.path.basename(path) == 'src':
                return os.path.dirname(path)
            path = os.path.dirname(path)
    return None


def data_dir(sub=''):
    """Carpeta de datos (se crea si no existe): bags, videos, trayectorias."""
    root = os.environ.get('BEBOP_DATA_DIR')
    if root:
        root = os.path.abspath(os.path.expanduser(root))
    else:
        root = package_source_dir()
        if root is None:
            root = os.path.expanduser(os.path.join('~', '.ros', PACKAGE))
            warnings.warn(
                f'No se encontró el código fuente de {PACKAGE}; los datos se '
                f'guardan en {root}. Define CONTROL_BEBOP_DIR para corregirlo.')
    path = os.path.join(root, sub) if sub else root
    os.makedirs(path, exist_ok=True)
    return path


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(PACKAGE)
    except Exception:
        return None


def config_file(name, prefer_source=True):
    """Ruta a config/<name>: primero en el código fuente y luego en share.

    Devuelve None si no existe en ninguno.
    """
    candidates = []
    src = package_source_dir()
    share = share_dir()
    if src:
        candidates.append(os.path.join(src, 'config', name))
    if share:
        candidates.append(os.path.join(share, 'config', name))
    if not prefer_source:
        candidates.reverse()
    return next((c for c in candidates if os.path.exists(c)), None)


def source_config_path(name):
    """Ruta donde ESCRIBIR config/<name> (código fuente si existe)."""
    src = package_source_dir()
    base = os.path.join(src, 'config') if src else data_dir('config')
    os.makedirs(base, exist_ok=True)
    return os.path.join(base, name)


def add_user_venvs(names=('rob_env', 'env_dron'), prepend=True):
    """Agrega a sys.path los site-packages de entornos virtuales del usuario.

    Solo se usan los de la versión de Python que está corriendo (3.10 en
    Humble, 3.12 en Jazzy): mezclar versiones rompe las importaciones.
    prepend=True les da prioridad sobre los paquetes del sistema.
    """
    ver = f'python{sys.version_info.major}.{sys.version_info.minor}'
    added = []
    for name in names:
        site = os.path.expanduser(os.path.join('~', name, 'lib', ver, 'site-packages'))
        if os.path.isdir(site) and site not in sys.path:
            if prepend:
                sys.path.insert(0, site)
            else:
                sys.path.append(site)
            added.append(site)
    return added
