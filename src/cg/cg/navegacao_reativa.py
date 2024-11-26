import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
import time

class NavegacaoReativa(Node):

    def __init__(self):
        super().__init__('navegacao_reativa')
        self.move_client = self.create_client(MoveCmd, '/move_command')

        self.posicao_visitada = set()
        self.pilha_exploracao = []
        self.ultima_direcao = None

    def requisicao_movimento(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        time.sleep(0.1)
        return result if result else self.get_logger().error("Erro ao obter resposta do serviço.")
    
    # Lógica da navegação reativa
    def movimento(self):
        while rclpy.ok():
            response = self.requisicao_movimento('')
            if not response:
                break

            robot_pos = tuple(response.robot_pos)
            target_pos = tuple(response.target_pos)
            sensors = {'left': response.left, 'down': response.down, 'up': response.up, 'right': response.right}

            self.posicao_visitada.add(robot_pos)

            # Mover na direção do alvo (Follow-the-Target)
            move_direction = self.encontrar_direcao(robot_pos, target_pos, sensors)

            # Backtracking (se necessário)
            if move_direction:
                move_result = self.requisicao_movimento(move_direction)
                if move_result.success:
                    self.get_logger().info(f'Movimento bem-sucedido para {move_direction}. Nova posição: {move_result.robot_pos}')
                    self.ultima_direcao = move_direction
                    if sum(1 for v in sensors.values() if v in ['f', 't']) > 1:
                        self.pilha_exploracao.append(robot_pos)
                continue

            if self.pilha_exploracao:
                self.get_logger().info("Sem movimentos válidos, voltando para uma posição anterior...")
                self.mover(self.pilha_exploracao.pop())
            else:
                self.get_logger().info("Nenhum caminho disponível. Encerrando.")
                break

    # Define a melhor direção com base no target e/ou acompanhando os obstaculos
    def encontrar_direcao(self, robot_pos, target_pos, sensors):
        directions = ['left', 'down', 'up', 'right']
        wall_follow_order = ['right', 'down', 'left', 'up']
        min_distance = float('inf')
        best_direction = None

        # 1 opção: se aproximar do alvo
        for direction, adjacent in zip(directions, [sensors[d] for d in directions]):
            if adjacent in ['f', 't']:
                new_pos = self.simular_movimento(robot_pos, direction)
                if tuple(new_pos) not in self.posicao_visitada:
                    distance = abs(new_pos[0] - target_pos[0]) + abs(new_pos[1] - target_pos[1])
                    if distance < min_distance:
                        min_distance = distance
                        best_direction = direction

        # 2 opção: contornar obstáculos (Boundary Following)
        if not best_direction:
            for direction in wall_follow_order:
                if sensors[direction] in ['f', 't'] and direction != self.direcao_oposta(self.ultima_direcao):
                    return direction

        return best_direction

    def simular_movimento(self, pos, direction):
        offsets = {'left': (0, -1), 'down': (1, 0), 'up': (-1, 0), 'right': (0, 1)}
        return [pos[0] + offsets[direction][0], pos[1] + offsets[direction][1]]

    def direcao_oposta(self, direction):
        opposites = {'left': 'right', 'right': 'left', 'up': 'down', 'down': 'up'}
        return opposites.get(direction)

    # Direção para se mover em direção à posição de backtracking
    def mover(self, target_pos):
        while rclpy.ok():
            response = self.requisicao_movimento('')
            robot_pos = tuple(response.robot_pos)
            if robot_pos == target_pos:
                return

            offsets = {'down': (1, 0), 'up': (-1, 0), 'right': (0, 1), 'left': (0, -1)}
            direction = next(
                (d for d, offset in offsets.items() if (robot_pos[0] + offset[0], robot_pos[1] + offset[1]) == target_pos),
                None
            )

            if direction:
                move_result = self.requisicao_movimento(direction)
                if not move_result.success:
                    self.get_logger().warning(f"Falha ao voltar para {direction}. Tentando outra direção.")
                    break

def main():
    rclpy.init()
    navigator = NavegacaoReativa()
    navigator.movimento()
    rclpy.shutdown()

if __name__ == '__main__':
    main()