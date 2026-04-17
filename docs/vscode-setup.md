# Configuracao do VS Code

## Dev Container (IntelliSense para ROS2)

O projeto inclui uma configuracao de Dev Container em `.devcontainer/devcontainer.json` que configura o VS Code para trabalhar dentro do container Docker com IntelliSense completo.

### Como usar

1. Instale a extensao [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) no VS Code.
2. Abra o projeto no VS Code.
3. Quando aparecer a notificacao "Reopen in Container", clique nela. Ou use `Ctrl+Shift+P` > "Dev Containers: Reopen in Container".
4. O VS Code vai conectar no container `ssl-vice` com Python e Pylance configurados.

### O que ja vem configurado

- **Pylance** com paths para pacotes ROS2 (`rclpy`, `std_srvs`, etc.)
- **system_interfaces** (mensagens e servicos gerados) reconhecidos pelo IntelliSense
- Extensoes Python e Pylance instaladas automaticamente no container

### Observacoes

- E necessario ter feito `vice build` pelo menos uma vez antes de abrir o Dev Container, para que as mensagens em `install/system_interfaces/` existam.
- Erros de tipo em classes protobuf (ex: `grSim_Robot_Command`) sao esperados — protobuf gera atributos dinamicamente e o Pylance nao consegue resolve-los estaticamente.

## Ruff (Linting e Formatacao)

O projeto usa [Ruff](https://docs.astral.sh/ruff/) para linting e formatacao de codigo Python. As regras estao definidas em `ruff.toml` na raiz do projeto.

### Instalacao

Instale a extensao [Ruff](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff) no VS Code (por Charlie Marsh).

### Formatacao automatica ao salvar

Para configurar o Ruff como formatador padrao com auto-format ao salvar, adicione ao seu `settings.json` do VS Code (`Ctrl+Shift+P` > "Preferences: Open User Settings (JSON)"):

```json
"[python]": {
    "editor.defaultFormatter": "charliermarsh.ruff",
    "editor.formatOnSave": true,
    "editor.codeActionsOnSave": {
        "source.fixAll.ruff": "explicit",
        "source.organizeImports.ruff": "explicit"
    }
}
```

Isso vai:
- Formatar o codigo automaticamente ao salvar
- Corrigir problemas de lint auto-corrigiveis
- Organizar imports automaticamente

### Uso manual

- `Ctrl+Shift+P` > "Ruff: Format Document" para formatar um arquivo
- Problemas de lint aparecem sublinhados automaticamente no editor
