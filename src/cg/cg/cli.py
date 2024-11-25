import typer
import inquirer
import rclpy

from .navegacao_reativa import NavegacaoReativa
from .navegacao_mapeada import NavegacaoMapeada  

app = typer.Typer()

def resposta_cli(resposta):
    try:
        if resposta["opcao"] == 'Navegação Reativa':
            typer.echo('Iniciando navegação reativa.')
            rclpy.init()
            try:
                navegacao_reativa = NavegacaoReativa()
                navegacao_reativa.movimento()
            except KeyboardInterrupt:
                print('\nExecução interrompida pelo usuário.')
            finally:
                if rclpy.ok():
                    rclpy.shutdown()

        elif resposta['opcao'] == 'Navegação Mapeada':
            typer.echo('Iniciando navegação mapeada.')
            rclpy.init()
            try:
                navegacao_mapeada = NavegacaoMapeada()
                navegacao_mapeada.movimento()
            except KeyboardInterrupt:
                print('\nExecução interrompida pelo usuário.')
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
    except Exception as e:
        typer.echo(f"Ocorreu um erro: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

@app.command()
def cli():
    opcoes = [
        inquirer.List(
            "opcao",
            message='Escolha um método de navegação:',
            choices=['Navegação Reativa', 'Navegação Mapeada'],
        ),
    ]
    resposta = inquirer.prompt(opcoes)

    if resposta: 
        resposta_cli(resposta)
    else:
        typer.echo("Nenhuma opção foi escolhida.")

def main():
    app()
