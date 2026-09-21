import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union


LatLon = Tuple[float, float]


def _format_point(p: Union[str, LatLon]) -> str:
    if isinstance(p, str):
        return p
    return f"{p[0]},{p[1]}"


def decode_polyline(polyline_str: str) -> List[LatLon]:
    """Decodes a Google encoded polyline into [(lat, lon), ...]."""
    index = 0
    lat = 0
    lon = 0
    coordinates: List[LatLon] = []

    while index < len(polyline_str):
        shift = 0
        result = 0
        while True:
            b = ord(polyline_str[index]) - 63
            index += 1
            result |= (b & 0x1F) << shift
            shift += 5
            if b < 0x20:
                break
        dlat = ~(result >> 1) if (result & 1) else (result >> 1)
        lat += dlat

        shift = 0
        result = 0
        while True:
            b = ord(polyline_str[index]) - 63
            index += 1
            result |= (b & 0x1F) << shift
            shift += 5
            if b < 0x20:
                break
        dlon = ~(result >> 1) if (result & 1) else (result >> 1)
        lon += dlon

        coordinates.append((lat / 1e5, lon / 1e5))

    return coordinates


@dataclass
class DirectionsCacheEntry:
    created_t: float
    route: List[LatLon]


class GoogleMapsDirections:
    """Google Directions API client with rate limiting + caching."""

    def __init__(
        self,
        api_key: Optional[str],
        min_interval_s: float = 15.0,
        cache_ttl_s: float = 300.0,
        cache_path: Optional[str] = "/tmp/robotx_directions_cache.json",
    ) -> None:
        self.api_key = api_key
        self.min_interval_s = float(min_interval_s)
        self.cache_ttl_s = float(cache_ttl_s)
        self.cache_path = Path(cache_path) if cache_path else None

        self._mem_cache: Dict[str, DirectionsCacheEntry] = {}
        self._last_call_t = 0.0
        self._load_cache()

    def _cache_key(self, start: Union[str, LatLon], destination: Union[str, LatLon]) -> str:
        return f"{_format_point(start)}->{_format_point(destination)}"

    def _load_cache(self) -> None:
        if self.cache_path is None or not self.cache_path.exists():
            return
        try:
            data = json.loads(self.cache_path.read_text())
            now = time.time()
            for k, v in data.items():
                created_t = float(v.get("created_t", 0))
                if now - created_t > self.cache_ttl_s:
                    continue
                route = [(float(a), float(b)) for a, b in v.get("route", [])]
                self._mem_cache[k] = DirectionsCacheEntry(created_t=created_t, route=route)
        except Exception:
            pass

    def _persist_cache(self) -> None:
        if self.cache_path is None:
            return
        try:
            payload = {
                k: {"created_t": e.created_t, "route": e.route}
                for k, e in self._mem_cache.items()
            }
            self.cache_path.write_text(json.dumps(payload))
        except Exception:
            pass

    def _get_cached(self, key: str) -> Optional[List[LatLon]]:
        entry = self._mem_cache.get(key)
        if entry is None:
            return None
        if time.time() - entry.created_t > self.cache_ttl_s:
            return None
        return entry.route

    async def get_route(self, start: Union[str, LatLon], destination: Union[str, LatLon]) -> List[LatLon]:
        if not self.api_key:
            raise RuntimeError("Google Maps API key missing (ROBOTX_GOOGLE_MAPS_API_KEY)")

        key = self._cache_key(start, destination)
        cached = self._get_cached(key)
        if cached is not None:
            return cached

        now = time.time()
        if self._last_call_t and (now - self._last_call_t) < self.min_interval_s:
            # Rate limit: if no cached route exists, fail fast.
            raise RuntimeError("Directions API rate limited; try again later")

        self._last_call_t = now

        origin = _format_point(start)
        dest = _format_point(destination)

        url = "https://maps.googleapis.com/maps/api/directions/json"
        params = {
            "origin": origin,
            "destination": dest,
            "mode": "driving",
            "key": self.api_key,
            "alternatives": "false",
        }

        import httpx

        async with httpx.AsyncClient(timeout=10.0) as client:
            resp = await client.get(url, params=params)
            resp.raise_for_status()
            data = resp.json()

        routes = data.get("routes", [])
        if not routes:
            raise RuntimeError(f"Directions API returned no routes: {data.get('status')} {data.get('error_message')}")

        poly = routes[0].get("overview_polyline", {}).get("points")
        if not poly:
            raise RuntimeError("Directions API missing overview_polyline")

        route = decode_polyline(poly)
        self._mem_cache[key] = DirectionsCacheEntry(created_t=time.time(), route=route)
        self._persist_cache()
        return route
