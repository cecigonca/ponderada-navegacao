import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
import numpy as np


class NavegacaoReativa(Node):
    def __init__(self):
        super().__init__('navegacao_reativa')
        self.client = self.create_client(MoveCmd, '/move_command')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Aguardando pelo serviço /move_command...")

        # Ordem de prioridade para as direções
        self.prioridade_direcoes = ["up", "right", "down", "left"]

        # pilha que armazena as posições que tomam a decisão para evitar o loop
        self.backtrack_stack = []

    def mover(self, direction):
        self.req = MoveCmd.Request()
        self.req.direction = direction
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)  # Aguarda o futuro ser concluído
        if future.result() is not None:
            return future.result()  # Retorna o resultado do serviço
        else:
            self.get_logger().error("Erro: O serviço não retornou um resultado.")
            return None

    def movimento(self):
        while rclpy.ok():
            response = self.mover("any")
            
            if response.success:
                # Verifica se o robô alcançou o alvo
                if np.array_equal(response.robot_pos, response.target_pos):
                    self.get_logger().info("Alvo alcançado!")
                    break

                # Obtem as informações dos sensores
                sensores = {
                    "left": response.left,
                    "down": response.down,
                    "up": response.up,
                    "right": response.right,
                }

                # Decide a próxima direção com base nas prioridades
                direcao = self.decidir_direcao(sensores)
                self.get_logger().info(f"Movendo para: {direcao}")

                # Move o robô na direção escolhida
                movimento_resposta = self.mover(direcao)
                if not movimento_resposta.success:
                    self.get_logger().error(f"Erro ao mover para {direcao}.")
            else:
                self.get_logger().error("Erro ao se comunicar com o serviço.")

    def decidir_direcao(self, sensores):
        for direcao in self.prioridade_direcoes:
            if sensores[direcao] == "t":  # Prioriza o alvo
                return direcao
        for direcao in self.prioridade_direcoes:
            if sensores[direcao] == "f":  # Movimenta para espaços livres
                return direcao
        return "left"  # Fallback para evitar travamento
    
    def main(args=None):
        if not rclpy.ok():
            rclpy.init(args=args)
        
        
        # Cria o nó e adiciona ao executor
        navigator = NavegacaoReativa()
        
        try:
            # Inicia a navegação
            navigator.get_logger().info("Starting navigation...")
            navigator.nav()
        finally:
            navigator.destroy_node()

    # Ponto de entrada do programa
    if __name__ == '__main__':
        main()  