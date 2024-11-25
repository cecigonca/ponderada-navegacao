import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, GetMap
import numpy as np
import time

class NavegacaoMapeada(Node):
    def __init__(self):
        super().__init__('navegacao_mapeada')
        self.cliente_movimento = self.create_client(MoveCmd, '/move_command')
        self.cliente_mapa = self.create_client(GetMap, '/get_map')
        self._esperar_servico(self.cliente_movimento, '/move_command')
        self._esperar_servico(self.cliente_mapa, '/get_map')
        self.mapa = None
        self.caminho = []

    def _esperar_servico(self, cliente, nome_servico):
        while not cliente.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Aguardando pelo serviço {nome_servico}...')

    def obter_mapa(self):
        req = GetMap.Request()
        futuro = self.cliente_mapa.call_async(req)
        rclpy.spin_until_future_complete(self, futuro)
        resposta = futuro.result()
        if resposta:
            self.mapa = np.array(resposta.occupancy_grid_flattened, dtype=str).reshape(resposta.occupancy_grid_shape)
            inicio = tuple(np.argwhere(self.mapa == 'r')[0])
            objetivo = tuple(np.argwhere(self.mapa == 't')[0])
            self.mapa = np.where(self.mapa == 'f', 0, 1)
            self.get_logger().info(f"Mapa processado. Início: {inicio}, Alvo: {objetivo}")
            return inicio, objetivo
        else:
            self.get_logger().error("Falha ao obter o mapa.")
            return None, None

    def a_star(self, inicio, objetivo):
        def heuristica(a, b): return abs(a[0] - b[0]) + abs(a[1] - b[1])
        aberto, origem = {inicio}, {}
        g_score, f_score = {inicio: 0}, {inicio: heuristica(inicio, objetivo)}

        while aberto:
            atual = min(aberto, key=lambda x: f_score.get(x, float('inf')))
            if atual == objetivo:
                caminho = []
                while atual in origem:
                    caminho.append(atual)
                    atual = origem[atual]
                return caminho[::-1]
            aberto.remove(atual)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                vizinho = (atual[0] + dx, atual[1] + dy)
                if 0 <= vizinho[0] < self.mapa.shape[0] and 0 <= vizinho[1] < self.mapa.shape[1] and self.mapa[vizinho] == 0:
                    novo_custo = g_score[atual] + 1
                    if vizinho not in g_score or novo_custo < g_score[vizinho]:
                        origem[vizinho] = atual
                        g_score[vizinho] = novo_custo
                        f_score[vizinho] = novo_custo + heuristica(vizinho, objetivo)
                        aberto.add(vizinho)
        return []

    def navegar(self, caminho):
        for atual, destino in zip(caminho, caminho[1:]):
            direcao = self.calcular_direcao(atual, destino)
            if not self._mover(direcao):
                self.get_logger().error("Falha ao mover o robô.")
                break
            time.sleep(0.5)

    def calcular_direcao(self, atual, destino):
        dx, dy = destino[0] - atual[0], destino[1] - atual[1]
        return 'up' if dx == -1 else 'down' if dx == 1 else 'left' if dy == -1 else 'right'

    def _mover(self, direcao):
        req = MoveCmd.Request()
        req.direction = direcao
        futuro = self.cliente_movimento.call_async(req)
        rclpy.spin_until_future_complete(self, futuro)
        resposta = futuro.result()
        self.get_logger().info(f"Movimento {direcao}: sucesso={resposta.success}")
        return resposta.success

def main(args=None):
    rclpy.init(args=args)
    nodo = NavegacaoMapeada()
    inicio, objetivo = nodo.obter_mapa()
    if inicio and objetivo:
        caminho = nodo.a_star(inicio, objetivo)
        if caminho:
            nodo.navegar(caminho)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()