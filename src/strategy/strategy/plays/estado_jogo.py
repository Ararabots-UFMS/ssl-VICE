"""UMA assinatura de 'game_state' para toda a arvore.

POR QUE ISTO EXISTE
-------------------
Cada folha da arvore criava a SUA propria assinatura de 'game_state'. Eram 18,
e o topico publica a ~58 Hz: mais de mil callbacks por segundo, cada um
desserializando a mensagem inteira - todos os robos, a bola, o campo e o
arbitro.

O custo apareceu na medicao: o strategyNode consumia 109% de CPU (um nucleo
inteiro) e o ciclo de 10 Hz pedido em 'create_timer(0.1, ...)' entregava 6,4 Hz.
Como o MovementManager SUBSTITUI a lista de alvos a cada mensagem, um ciclo
lento significa alvo velho no planejador por mais tempo - parte da lentidao dos
robos em campo vem daqui, nao da cadeia de movimento.

Com uma assinatura so, a desserializacao acontece UMA vez por mensagem e as
folhas recebem a mesma referencia por chamada de funcao Python, que e barata.
O comportamento nao muda: cada folha continua com o seu proprio callback e os
seus proprios campos.
"""

from system_interfaces.msg._game_state import GameState


class EstadoJogo:
    """Distribui o ultimo 'game_state' para quem se registrar."""

    _callbacks = []
    _assinado = False
    _ultimo = None

    @classmethod
    def registrar(cls, node, callback):
        """Troca um 'create_subscription' por um registro aqui.

        O primeiro a registrar e quem cria a assinatura de verdade; o node
        escolhido nao importa, porque todos vivem no mesmo processo e no mesmo
        executor.
        """
        cls._callbacks.append(callback)
        if cls._ultimo is not None:
            # quem chega depois nao espera a proxima mensagem
            callback(cls._ultimo)
        if not cls._assinado:
            cls._assinado = True
            node.create_subscription(GameState, "game_state",
                                     cls._distribuir, 10)

    @classmethod
    def _distribuir(cls, msg):
        cls._ultimo = msg
        for cb in cls._callbacks:
            cb(msg)

    @classmethod
    def ultimo(cls):
        return cls._ultimo
