---
id: setup
title: Prepare the Shell
---

# Preface

We recommend to use zsh. This is not mandatory and if you have strongly prefer another shell feel free to do so. But this requires you to adapt the following setup and install instructions.

We will also make use of some extensions to make working with the shell more convenient.

Let’s install some requirements.

Follow the step-by-step setup instructions provided during the first session.

---

# Installation

:::note
After the following command you will be asked if you want to use zsh as your default shell. We recommend to confirm this.
:::

```bash
sudo apt install -y zsh git curl wget vim \
&& sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
```
And now set up our shell real quick:

```bash
mkdir -p "$HOME/.zsh" \
&& git clone https://github.com/sindresorhus/pure.git "$HOME/.zsh/pure"\
&& echo 'fpath+=$HOME/.zsh/pure \nautoload -U promptinit; promptinit \nprompt pure' | cat - ~/.zshrc > temp && mv temp ~/.zshrc \
&& sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME=""/' ~/.zshrc \
&& git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
&& sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions)/' ~/.zshrc \
&& echo "zstyle ':prompt:pure:path' color 075\nzstyle ':prompt:pure:prompt:success' color 214\nzstyle ':prompt:pure:user' color 119\nzstyle ':prompt:pure:host' color 119\nZSH_AUTOSUGGEST_HIGHLIGHT_STYLE='fg=161'" >> ~/.zshrc \
&& echo "zstyle ':prompt:pure:path' color 075\nzstyle ':prompt:pure:prompt:success' color 214\nzstyle ':prompt:pure:user' color 119\nzstyle ':prompt:pure:host' color 119\nZSH_AUTOSUGGEST_HIGHLIGHT_STYLE='fg=161'" >> ~/.zshrc \
&& echo "export TERM=xterm-256color" >> ~/.zshrc \
&& source ~/.zshrc
```
---

# Useful Hints

## Auto Suggestions
As soon as you start to type something that matches any previously executed command, the shell will provide you with a suggested command. It is highlighted in a different color and it is only a virtual text up until now. To actually apply the suggestion we use the right arrow key. If we do not care about the suggestion, just keep typing.

## Traversing the History
The “going-back-in-history” behaviour of our shell is quite convenient. It considers what we have already entered when pressing Up. This is useful, because usually we know how a command that we have previously executed starts, but to not remember the whole thing. Consider the following example: We have already echoed a specific topic in the past. Ofcourse we know that the command starts with ros2 topic echo. But unfortunately we do not remember the very complicated topic name and don’t want to invest the time to look it up. Since we already had executed the command before, we can traverse the shell’s history to get the command.

## Auto Complete
Oh yeah, auto complete is a feature we should extensively make use of. With Tab we trigger the completion. If it can be performed because the completion is unambigious, it will be done. Otherwise we will immediately be provided with the available options. We can navigate through them with Tab and Shift + Tab. If there are many available options, they will be arranged in a table format. We can navigate to the desired completion directly with the arrow keys.

:::note
Tab is your best friend. Use it as often as possible. Never type anything by hand if you can do an auto completion instead.
:::