import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, GetMap
import time
import heapq
import numpy as np

class NavegacaoMapeada(Node):

    def __init__(self):
        super().__init__('navegacao_mapeada')
        self.cliente_movimento = self.create_client(MoveCmd, '/move_command')
        self.cliente_mapa = self.create_client(GetMap, '/get_map')
        self.mapa = None
        self.posicao_inicial = None
        self.posicao_alvo = None

    def requisitar_movimento(self, direcao):
        requisicao = MoveCmd.Request()
        requisicao.direction = direcao
        futuro = self.cliente_movimento.call_async(requisicao)
        rclpy.spin_until_future_complete(self, futuro)
        resultado = futuro.result()
        time.sleep(0.2)
        if not resultado:
            self.get_logger().error("Erro ao obter resposta do serviço de movimento.")
        return resultado

    def obter_mapa(self):
        futuro = self.cliente_mapa.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, futuro)
        resposta = futuro.result()

        if not resposta:
            self.get_logger().error("Falha ao obter o mapa.")
            return False

        formato_mapa = tuple(resposta.occupancy_grid_shape)
        dados_mapa = resposta.occupancy_grid_flattened
        self.mapa = np.array(dados_mapa, dtype=str).reshape(formato_mapa)

        pos_inicial = np.argwhere(self.mapa == 'r')
        pos_alvo = np.argwhere(self.mapa == 't')

        if len(pos_inicial) == 0 or len(pos_alvo) == 0:
            self.get_logger().warning("Posições inicial ou alvo não encontradas no mapa.")
            return False

        self.posicao_inicial, self.posicao_alvo = tuple(pos_inicial[0]), tuple(pos_alvo[0])
        self.mapa = np.where(self.mapa == 'f', 0, 1)
        self.mapa[self.posicao_inicial] = 0
        self.mapa[self.posicao_alvo] = 0
        self.get_logger().info(f"Posição inicial: {self.posicao_inicial}, Alvo: {self.posicao_alvo}")
        return True

    def a_star(self, inicio, objetivo):
        def heuristica_manhattan(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        conjunto_aberto = [(0, inicio)]
        origem, g_score = {}, {inicio: 0}

        while conjunto_aberto:
            _, atual = heapq.heappop(conjunto_aberto)
            if atual == objetivo:
                caminho = []
                while atual in origem:
                    atual, direcao = origem[atual]
                    caminho.append(direcao)
                return caminho[::-1]

            for direcao, vizinho in self.obter_vizinhos(atual):
                if self.mapa[vizinho] == 1:
                    continue

                custo_tentativo = g_score[atual] + 1
                if vizinho not in g_score or custo_tentativo < g_score[vizinho]:
                    origem[vizinho] = (atual, direcao)
                    g_score[vizinho] = custo_tentativo
                    heapq.heappush(conjunto_aberto, (custo_tentativo + heuristica_manhattan(vizinho, objetivo), vizinho))

        self.get_logger().warning("Nenhum caminho encontrado.")
        return []

    def obter_vizinhos(self, posicao):
        direcoes = {
            'cima': (-1, 0), 'baixo': (1, 0),
            'esquerda': (0, -1), 'direita': (0, 1)
        }
        return [
            (direcao, (posicao[0] + di, posicao[1] + dj))
            for direcao, (di, dj) in direcoes.items()
            if 0 <= posicao[0] + di < self.mapa.shape[0] and 0 <= posicao[1] + dj < self.mapa.shape[1]
        ]

    def movimento(self):
        if not self.obter_mapa():
            self.get_logger().warning("Abortando navegação devido à falha ao obter o mapa.")
            return

        caminho = self.a_star(self.posicao_inicial, self.posicao_alvo)
        if not caminho:
            self.get_logger().warning("Nenhum caminho planejado. Abortando navegação.")
            return

        self.get_logger().info(f"Caminho planejado: {caminho}")

        for direcao in caminho:
            resposta = self.requisitar_movimento(direcao)
            if resposta and resposta.success:
                self.get_logger().info(f'Movimento bem-sucedido para {direcao}. Nova posição: {resposta.robot_pos}')
            else:
                self.get_logger().warning("Falha ao mover o robô.")
                break

def main():
    rclpy.init()
    navegador = NavegacaoMapeada()
    navegador.movimento()
    rclpy.shutdown()

if __name__ == '__main__':
    main()